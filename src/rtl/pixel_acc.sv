// ============================================================
//  pixel_acc - pipelined Sobel edge detector (line-buffer version)
//
//  Throughput: ~1 output word per clock cycle
//  (about (H+1)*(W+1) = 289*89 = 25.7k cycles per frame)
//
//  Pipeline (one "beat" per clock, tagged with its position):
//
//    ISSUE -> s1 (addr regs) -> A (data + line buffers)
//          -> B (window regs) -> C (dx/dy regs) -> D (obuf regs)
//
//  The input is read strictly in raster order, ONE word per beat.
//  Two line buffers hold the previous two rows, so the 3-row
//  window is built from: lb1 (row y-2), lb0 (row y-1), input (row y).
//  When row y arrives, output row y-1 is computed.
//
//  Each row phase has IMG_WIDTH+1 beats: IMG_WIDTH real words plus
//  one flush beat (no read) so that the last column of the row can
//  be computed before the next row's first word shifts in.
//  There are IMG_HEIGHT+1 row phases: phase 0 only fills lb0,
//  phase IMG_HEIGHT has no reads (bottom border replication).
// ============================================================
module pixel_acc #(
    parameter int BUF_AW    = 15,
    parameter int BUF_DEPTH = 25344,
    parameter int IMG_WIDTH = 88      // 352 pixels / 4 pixels-per-word
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // ibuf read port
    output logic                  ibuf_rd_en,
    output logic [BUF_AW-1:0]     ibuf_rd_addr,
    input  logic [31:0]           ibuf_rd_data,
    input  logic [BUF_AW:0]       ibuf_count,

    // obuf write port
    output logic                  obuf_wr_en,
    output logic [BUF_AW-1:0]     obuf_wr_addr,
    output logic [31:0]           obuf_wr_data,

    // handshake
    input  logic                  start,
    output logic                  finish
);

    localparam int IMG_HEIGHT = BUF_DEPTH / IMG_WIDTH;     // 288 rows
    localparam int CNTW       = $clog2(IMG_WIDTH + 1);
    localparam int ROWW       = $clog2(IMG_HEIGHT + 1);

    // ---------------- ISSUE stage ----------------
   
    logic              run;
    logic [CNTW-1:0]   i_col;          // 0..IMG_WIDTH (IMG_WIDTH = flush beat)
    logic [ROWW-1:0]   i_row;          // 0..IMG_HEIGHT
    logic [BUF_AW-1:0] rd_cnt;         // next ibuf word to read
    logic [BUF_AW-1:0] wr_cnt;         // next obuf word to write
    logic need_word, can_issue;
	always_comb begin
    		need_word = (i_row < ROWW'(IMG_HEIGHT)) && (i_col < CNTW'(IMG_WIDTH));
    		can_issue = !need_word || ({1'b0, rd_cnt} < ibuf_count);
	end

    // ---------------- tags travelling with the data ----------------
    logic              s1_v;           // beat tag, aligned with ibuf_rd_addr
    logic [CNTW-1:0]   s1_col;
    logic [ROWW-1:0]   s1_row;

    logic              a_v;            // stage A: ibuf_rd_data is valid now
    logic [CNTW-1:0]   a_col;
    logic [ROWW-1:0]   a_row;

    // ---------------- line buffers (no reset) ----------------
    logic [31:0] lb0 [IMG_WIDTH];      // row y-1
    logic [31:0] lb1 [IMG_WIDTH];      // row y-2

    logic [CNTW-1:0] a_idx;
    logic [31:0]     a_top, a_mid, a_bot;

    always_comb begin
        a_idx = (a_col == CNTW'(IMG_WIDTH)) ? CNTW'(IMG_WIDTH - 1) : a_col;
        a_mid = lb0[a_idx];
        // top border: row above row 0 is row 0
        a_top = (a_row == ROWW'(1)) ? a_mid : lb1[a_idx];
        // bottom border: row below the last row is the last row
        a_bot = (a_row == ROWW'(IMG_HEIGHT)) ? a_mid : ibuf_rd_data;
    end

    always_ff @(posedge clk) begin
        if (a_v && a_col != CNTW'(IMG_WIDTH)) begin
            lb1[a_idx] <= lb0[a_idx];
            lb0[a_idx] <= a_bot;
        end
    end

    // ---------------- stage B: window registers ----------------
    // index = row: 0 = above, 1 = current, 2 = below
    logic [31:0]     w_l [3];          // word column w-1
    logic [31:0]     w_c [3];          // word column w   (being computed)
    logic [31:0]     w_n [3];          // word column w+1
    logic            b_v, b_last;
    logic [CNTW-1:0] b_w;

    logic [7:0] p [3][6];              // per row: [0]=left [1..4]=word [5]=right
    always_comb begin
        for (int r = 0; r < 3; r++) begin
            p[r][0] = (b_w == '0)                   ? w_c[r][7:0]   : w_l[r][31:24];
            p[r][1] = w_c[r][7:0];
            p[r][2] = w_c[r][15:8];
            p[r][3] = w_c[r][23:16];
            p[r][4] = w_c[r][31:24];
            p[r][5] = (b_w == CNTW'(IMG_WIDTH - 1)) ? w_c[r][31:24] : w_n[r][7:0];
        end
    end

    // ---------------- stage C: Sobel partial derivatives ----------------
    logic        c_v, c_last;
    logic [11:0] dx [4];
    logic [11:0] dy [4];

    // ---------------- stage D: obuf registers ----------------
    logic        d_last;

    // |a| + |b|, saturated to 8 bit
    function automatic logic [7:0] mag(input logic [11:0] a, input logic [11:0] b);
        logic [11:0] aa, ab, s;
        aa = a[11] ? (~a + 12'd1) : a;
        ab = b[11] ? (~b + 12'd1) : b;
        s  = aa + ab;
        return (s > 12'd255) ? 8'hFF : s[7:0];
    endfunction
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            run          <= 1'b0;
            i_col        <= '0;
            i_row        <= '0;
            rd_cnt       <= '0;
            wr_cnt       <= '0;
            ibuf_rd_en   <= 1'b0;
            ibuf_rd_addr <= '0;
            obuf_wr_en   <= 1'b0;
            obuf_wr_addr <= '0;
            obuf_wr_data <= '0;
            finish       <= 1'b0;
            s1_v   <= 1'b0;  s1_col <= '0;  s1_row <= '0;
            a_v    <= 1'b0;  a_col  <= '0;  a_row  <= '0;
            b_v    <= 1'b0;  b_last <= 1'b0; b_w    <= '0;
            c_v    <= 1'b0;  c_last <= 1'b0;
            d_last <= 1'b0;
            for (int r = 0; r < 3; r++) begin
                w_l[r] <= '0;
                w_c[r] <= '0;
                w_n[r] <= '0;
            end
            for (int j = 0; j < 4; j++) begin
                dx[j] <= '0;
                dy[j] <= '0;
            end
        end else begin
            // defaults: single-cycle pulses
            ibuf_rd_en <= 1'b0;
            obuf_wr_en <= 1'b0;
            finish     <= 1'b0;
            s1_v       <= 1'b0;

            // ---------- ISSUE ----------
            if (start && !run) begin
                run    <= 1'b1;
                i_col  <= '0;
                i_row  <= '0;
                rd_cnt <= '0;
                wr_cnt <= '0;
            end else if (run && can_issue) begin      
                s1_v   <= 1'b1;
                s1_col <= i_col;
                s1_row <= i_row;

                if (i_row < ROWW'(IMG_HEIGHT) && i_col < CNTW'(IMG_WIDTH)) begin
                    ibuf_rd_en   <= 1'b1;
                    ibuf_rd_addr <= rd_cnt;
                    rd_cnt       <= rd_cnt + 1'b1;
                end

                if (i_col == CNTW'(IMG_WIDTH)) begin
                    i_col <= '0;
                    if (i_row == ROWW'(IMG_HEIGHT)) run   <= 1'b0;
                    else                            i_row <= i_row + 1'b1;
                end else begin
                    i_col <= i_col + 1'b1;
                end
            end

            // ---------- s1 -> A (BRAM read latency) ----------
            a_v   <= s1_v;
            a_col <= s1_col;
            a_row <= s1_row;

            // ---------- A -> B: shift the window ----------
            // output (row a_row-1, word a_col-1) is valid from a_row>=1, a_col>=1
            b_v    <= a_v && (a_col != '0) && (a_row != '0);
            b_w    <= a_col - 1'b1;
            b_last <= a_v && (a_row == ROWW'(IMG_HEIGHT)) && (a_col == CNTW'(IMG_WIDTH));
            if (a_v) begin
                w_l <= w_c;
                w_c <= w_n;
                w_n <= '{a_top, a_mid, a_bot};
            end

            // ---------- B -> C: Sobel gradients ----------
            c_v    <= b_v;
            c_last <= b_last;
            if (b_v) begin
                for (int j = 0; j < 4; j++) begin
                    dx[j] <= ({4'b0, p[0][j+2]} - {4'b0, p[0][j]})
                          + (({4'b0, p[1][j+2]} - {4'b0, p[1][j]}) << 1)
                          +  ({4'b0, p[2][j+2]} - {4'b0, p[2][j]});
                    dy[j] <= ({4'b0, p[0][j]} + ({4'b0, p[0][j+1]} << 1) + {4'b0, p[0][j+2]})
                          -  ({4'b0, p[2][j]} + ({4'b0, p[2][j+1]} << 1) + {4'b0, p[2][j+2]});
                end
            end

            // ---------- C -> D: magnitude + write ----------
            d_last <= c_v && c_last;
            if (c_v) begin
                for (int j = 0; j < 4; j++)
                    obuf_wr_data[8*j +: 8] <= mag(dx[j], dy[j]);
                obuf_wr_en   <= 1'b1;
                obuf_wr_addr <= wr_cnt;
                wr_cnt       <= wr_cnt + 1'b1;
            end

            // finish one cycle after the last write enable is visible
            finish <= d_last;
        end
    end
endmodule
