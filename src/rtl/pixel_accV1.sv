// ============================================================
//  pixel_acc - STUDENT MODULE
//
//  You are given direct read access to ibuf and write access
//  to obuf. Both are synchronous BRAMs with 1-cycle read latency.
//
//  Interface:
//    start  - pulsed high for 1 cycle to begin processing
//    finish - assert high for 1 cycle when all output words
//             have been written to obuf
//
//  ibuf read port:
//    Set ibuf_rd_en=1 and ibuf_rd_addr to the word you want.
//    ibuf_rd_data is valid on the NEXT clock cycle.
//
//  obuf write port:
//    Set obuf_wr_en=1, obuf_wr_addr, obuf_wr_data on the same cycle.
//    The word is written on the rising edge.
//
//  Parameters:
//    BUF_AW    - address width (ibuf/obuf have 2^BUF_AW words)
//    BUF_DEPTH - total number of words in each buffer
//    IMG_WIDTH - frame width in words (= pixels per row / 4)
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

    // obuf write port
    output logic                  obuf_wr_en,
    output logic [BUF_AW-1:0]     obuf_wr_addr,
    output logic [31:0]           obuf_wr_data,

    // handshake
    input  logic                  start,
    output logic                  finish
);

    // ============================================================
    // BRAM has 1-cycle read latency, so:
    //   cycle 0: read ibuf[0] issued  (rd_data not yet valid)
    //   cycle 1: read ibuf[1] issued, ibuf[0] data valid -> write obuf[0]
    //   ...
    //   cycle N: read ibuf[N] issued (last), write obuf[N-1]
    //   cycle N+1: no new read, write obuf[N], assert finish
    //
    // ============================================================

    localparam int IMG_HEIGHT = BUF_DEPTH / IMG_WIDTH;     // 288 rows
    localparam int CNTW       = $clog2(IMG_WIDTH + 1);
    localparam int ROWW       = $clog2(IMG_HEIGHT + 1);

    typedef enum logic [2:0] {
        IDLE   = 3'd0,
        FETCH  = 3'd1,
        COMP1  = 3'd2,
        COMP2  = 3'd3,
        FINISH = 3'd4
    } state_t;

    state_t            state;
    logic [2:0]        k;            // step inside FETCH (0..4)
    logic [ROWW-1:0]   row;          // output row
    logic [CNTW-1:0]   w;            // output word column
    logic [CNTW-1:0]   fc, fcc;      // word column to fetch (fcc = clamped)
    logic [BUF_AW-1:0] base [3];     // word address of row above / current / below
    logic [7:0]        l_pix  [3];   // pixel left of c_word
    logic [31:0]       c_word [3];   // current word
    logic [31:0]       n_word [3];   // next word
    logic [11:0]       dx [4];       // two's complement partial derivatives
    logic [11:0]       dy [4];
    logic [7:0]        p  [3][6];    // per row: [0]=left [1..4]=word [5]=right
    logic [1:0]        ri;           // row being shifted in during FETCH (k-2)

    // ---- 6-pixel neighbourhood per row, with column replication ----
    always_comb begin
        ri  = 2'(k - 3'd2);
        fcc = (fc >= CNTW'(IMG_WIDTH)) ? CNTW'(IMG_WIDTH - 1) : fc;
        for (int r = 0; r < 3; r++) begin
            p[r][0] = (w == '0)                    ? c_word[r][7:0]   : l_pix[r];
            p[r][1] = c_word[r][7:0];
            p[r][2] = c_word[r][15:8];
            p[r][3] = c_word[r][23:16];
            p[r][4] = c_word[r][31:24];
            p[r][5] = (w == CNTW'(IMG_WIDTH - 1))  ? c_word[r][31:24] : n_word[r][7:0];
        end
    end

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
            state        <= IDLE;
            k            <= '0;
            row          <= '0;
            w            <= '0;
            fc           <= '0;
            ibuf_rd_en   <= 1'b0;
            ibuf_rd_addr <= '0;
            obuf_wr_en   <= 1'b0;
            obuf_wr_addr <= '0;
            obuf_wr_data <= '0;
            finish       <= 1'b0;
            for (int r = 0; r < 3; r++) begin
                base[r]   <= '0;
                l_pix[r]  <= '0;
                c_word[r] <= '0;
                n_word[r] <= '0;
            end
            for (int j = 0; j < 4; j++) begin
                dx[j] <= '0;
                dy[j] <= '0;
            end
        end else begin
            ibuf_rd_en <= 1'b0;
            obuf_wr_en <= 1'b0;
            finish     <= 1'b0;

            case (state)
                IDLE: begin
                    if (start) begin
                        row     <= '0;
                        w       <= '0;
                        fc      <= '0;
                        k       <= '0;
                        base[0] <= '0;                    // row above row 0 -> row 0
                        base[1] <= '0;
                        base[2] <= BUF_AW'(IMG_WIDTH);    // row 1
                        state   <= FETCH;
                    end
                end

                // Fetch word 'fc' of the three rows.
                //  k=0..2 issue reads (registered), data returns 2 cycles later:
                //  k=2 -> row above, k=3 -> current row, k=4 -> row below
                FETCH: begin
                    k <= k + 3'd1;
                    if (k <= 3'd2) begin
                        ibuf_rd_en   <= 1'b1;
                        ibuf_rd_addr <= base[k[1:0]] + BUF_AW'(fcc);
                    end
                    if (k >= 3'd2) begin
                        // shift window of row (k-2): left <- cur <- next <- new word
                        l_pix [ri] <= c_word[ri][31:24];
                        c_word[ri] <= n_word[ri];
                        n_word[ri] <= ibuf_rd_data;
                    end
                    if (k == 3'd4) begin
                        k <= '0;
                        if (fc == '0) fc    <= CNTW'(1);   // priming: fetch word 1 too
                        else          state <= COMP1;
                    end
                end

                // Sobel partial derivatives for the 4 pixels of the word
                COMP1: begin
                    for (int j = 0; j < 4; j++) begin
                        dx[j] <= ({4'b0, p[0][j+2]} - {4'b0, p[0][j]})
                              + (({4'b0, p[1][j+2]} - {4'b0, p[1][j]}) << 1)
                              +  ({4'b0, p[2][j+2]} - {4'b0, p[2][j]});
                        dy[j] <= ({4'b0, p[0][j]} + ({4'b0, p[0][j+1]} << 1) + {4'b0, p[0][j+2]})
                              -  ({4'b0, p[2][j]} + ({4'b0, p[2][j+1]} << 1) + {4'b0, p[2][j+2]});
                    end
                    state <= COMP2;
                end

                // |Dx|+|Dy|, saturate, write word, advance
                COMP2: begin
                    for (int j = 0; j < 4; j++)
                        obuf_wr_data[8*j +: 8] <= mag(dx[j], dy[j]);
                    obuf_wr_en   <= 1'b1;
                    obuf_wr_addr <= base[1] + BUF_AW'(w);

                    k <= '0;
                    if (w == CNTW'(IMG_WIDTH - 1)) begin      // last word of the row
                        w  <= '0;
                        fc <= '0;
                        if (row == ROWW'(IMG_HEIGHT - 1)) begin
                            state <= FINISH;
                        end else begin
                            row     <= row + 1'b1;
                            base[0] <= base[1];
                            base[1] <= base[2];
                            if (row < ROWW'(IMG_HEIGHT - 2))  // clamp row below at the bottom
                                base[2] <= base[2] + BUF_AW'(IMG_WIDTH);
                            state <= FETCH;
                        end
                    end else begin
                        w     <= w + 1'b1;
                        fc    <= w + CNTW'(2);
                        state <= FETCH;
                    end
                end

                FINISH: begin
                    finish <= 1'b1;
                    state  <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
