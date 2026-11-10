/*
  Contributors:
    * Dovydas Liutkus (dovli@dtu.dk)
  Description:
    * pixel_acc - STUDENT MODULE (VHDL version)
    * Drop-in VHDL equivalent of pixel_acc.sv. Instantiated by the
    * SystemVerilog wrapper pixel_acc_vhd_wrapper.sv (module pixel_acc).
    *
    * You are given direct read access to ibuf and write access
    * to obuf. Both are synchronous BRAMs with 1-cycle read latency.
    *
    * Interface:
    *   start  - pulsed high for 1 cycle to begin processing
    *   finish - assert high for 1 cycle when all output words
    *            have been written to obuf
    *
    * ibuf read port:
    *   Set ibuf_rd_en='1' and ibuf_rd_addr to the word you want.
    *   ibuf_rd_data is valid on the NEXT clock cycle.
    *
    * obuf write port:
    *   Set obuf_wr_en='1', obuf_wr_addr, obuf_wr_data on the same cycle.
    *   The word is written on the rising edge.
    *
    * Generics:
    *   BUF_AW    - address width (ibuf/obuf have 2^BUF_AW words)
    *   BUF_DEPTH - total number of words in each buffer
    *   IMG_WIDTH - frame width in words (= pixels per row / 4)
    *
    * NOTE: VHDL-2008 (vcom -2008). Entity name differs from pixel_acc
    * so it can share a library with the SystemVerilog wrapper.
*/

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pixel_acc_vhd is
  generic (
    BUF_AW    : integer := 15;
    BUF_DEPTH : integer := 25344;
    IMG_WIDTH : integer := 88       -- 352 pixels / 4 pixels-per-word
  );
  port (
    clk          : in  std_logic;
    rst_n        : in  std_logic;

    -- ibuf read port
    ibuf_rd_en   : out std_logic;
    ibuf_rd_addr : out std_logic_vector(BUF_AW-1 downto 0);
    ibuf_rd_data : in  std_logic_vector(31 downto 0);

    -- obuf write port
    obuf_wr_en   : out std_logic;
    obuf_wr_addr : out std_logic_vector(BUF_AW-1 downto 0);
    obuf_wr_data : out std_logic_vector(31 downto 0);

    -- handshake
    start        : in  std_logic;
    finish       : out std_logic
  );
end entity pixel_acc_vhd;

architecture rtl of pixel_acc_vhd is

  constant IMG_HEIGHT : integer := BUF_DEPTH / IMG_WIDTH;     -- 288 rows

  type state_t    is (S_IDLE, S_FETCH, S_COMP1, S_COMP2, S_FINISH);
  type base_arr_t is array (0 to 2) of unsigned(BUF_AW-1 downto 0);
  type pix_arr_t  is array (0 to 2) of std_logic_vector(7 downto 0);
  type word_arr_t is array (0 to 2) of std_logic_vector(31 downto 0);
  type d_arr_t    is array (0 to 3) of signed(11 downto 0);
  type nbh_t      is array (0 to 2, 0 to 5) of unsigned(7 downto 0);

  signal state  : state_t;
  signal k      : integer range 0 to 4;               -- step inside FETCH
  signal row    : integer range 0 to IMG_HEIGHT-1;    -- output row
  signal w      : integer range 0 to IMG_WIDTH-1;     -- output word column
  signal fc     : integer range 0 to IMG_WIDTH;       -- word column to fetch
  signal fcc    : integer range 0 to IMG_WIDTH-1;     -- fc clamped to the row
  signal base   : base_arr_t;                         -- word address of row above / current / below
  signal l_pix  : pix_arr_t;                          -- pixel left of c_word
  signal c_word : word_arr_t;                         -- current word
  signal n_word : word_arr_t;                         -- next word
  signal dx, dy : d_arr_t;                            -- partial derivatives
  signal p      : nbh_t;                              -- per row: 0=left 1..4=word 5=right

  function s12(u : unsigned) return signed is
  begin
    return signed(resize(u, 12));
  end function;

  -- |a| + |b|, saturated to 8 bit
  function mag(a, b : signed) return std_logic_vector is
    variable s : signed(11 downto 0);
  begin
    s := abs(a) + abs(b);
    if s > 255 then
      return x"FF";
    else
      return std_logic_vector(s(7 downto 0));
    end if;
  end function;

begin

  -- ---- 6-pixel neighbourhood per row, with column replication ----
  fcc <= IMG_WIDTH-1 when fc >= IMG_WIDTH else fc;

  nbh : process (all)
  begin
    for r in 0 to 2 loop
      if w = 0 then
        p(r, 0) <= unsigned(c_word(r)(7 downto 0));
      else
        p(r, 0) <= unsigned(l_pix(r));
      end if;
      p(r, 1) <= unsigned(c_word(r)(7 downto 0));
      p(r, 2) <= unsigned(c_word(r)(15 downto 8));
      p(r, 3) <= unsigned(c_word(r)(23 downto 16));
      p(r, 4) <= unsigned(c_word(r)(31 downto 24));
      if w = IMG_WIDTH-1 then
        p(r, 5) <= unsigned(c_word(r)(31 downto 24));
      else
        p(r, 5) <= unsigned(n_word(r)(7 downto 0));
      end if;
    end loop;
  end process nbh;

  -- ---- FSM + datapath ----
  fsm : process (clk, rst_n)
  begin
    if rst_n = '0' then
      state        <= S_IDLE;
      k            <= 0;
      row          <= 0;
      w            <= 0;
      fc           <= 0;
      ibuf_rd_en   <= '0';
      ibuf_rd_addr <= (others => '0');
      obuf_wr_en   <= '0';
      obuf_wr_addr <= (others => '0');
      obuf_wr_data <= (others => '0');
      finish       <= '0';
      base         <= (others => (others => '0'));
      l_pix        <= (others => (others => '0'));
      c_word       <= (others => (others => '0'));
      n_word       <= (others => (others => '0'));
      dx           <= (others => (others => '0'));
      dy           <= (others => (others => '0'));
    elsif rising_edge(clk) then
      ibuf_rd_en <= '0';
      obuf_wr_en <= '0';
      finish     <= '0';

      case state is

        when S_IDLE =>
          if start = '1' then
            row     <= 0;
            w       <= 0;
            fc      <= 0;
            k       <= 0;
            base(0) <= (others => '0');                      -- row above row 0 -> row 0
            base(1) <= (others => '0');
            base(2) <= to_unsigned(IMG_WIDTH, BUF_AW);       -- row 1
            state   <= S_FETCH;
          end if;

        -- Fetch word 'fc' of the three rows.
        --  k=0..2 issue reads (registered), data returns 2 cycles later:
        --  k=2 -> row above, k=3 -> current row, k=4 -> row below
        when S_FETCH =>
          if k <= 2 then
            ibuf_rd_en   <= '1';
            ibuf_rd_addr <= std_logic_vector(base(k) + fcc);
          end if;
          if k >= 2 then
            -- shift window of row (k-2): left <- cur <- next <- new word
            l_pix (k-2) <= c_word(k-2)(31 downto 24);
            c_word(k-2) <= n_word(k-2);
            n_word(k-2) <= ibuf_rd_data;
          end if;
          if k = 4 then
            k <= 0;
            if fc = 0 then
              fc <= 1;                  -- priming: fetch word 1 too
            else
              state <= S_COMP1;
            end if;
          else
            k <= k + 1;
          end if;

        -- Sobel partial derivatives for the 4 pixels of the word
        when S_COMP1 =>
          for j in 0 to 3 loop
            dx(j) <= (s12(p(0, j+2)) - s12(p(0, j)))
                   + shift_left(s12(p(1, j+2)) - s12(p(1, j)), 1)
                   + (s12(p(2, j+2)) - s12(p(2, j)));
            dy(j) <= (s12(p(0, j)) + shift_left(s12(p(0, j+1)), 1) + s12(p(0, j+2)))
                   - (s12(p(2, j)) + shift_left(s12(p(2, j+1)), 1) + s12(p(2, j+2)));
          end loop;
          state <= S_COMP2;

        -- |Dx|+|Dy|, saturate, write word, advance
        when S_COMP2 =>
          for j in 0 to 3 loop
            obuf_wr_data(8*j+7 downto 8*j) <= mag(dx(j), dy(j));
          end loop;
          obuf_wr_en   <= '1';
          obuf_wr_addr <= std_logic_vector(base(1) + w);

          k <= 0;
          if w = IMG_WIDTH-1 then                  -- last word of the row
            w  <= 0;
            fc <= 0;
            if row = IMG_HEIGHT-1 then
              state <= S_FINISH;
            else
              row     <= row + 1;
              base(0) <= base(1);
              base(1) <= base(2);
              if row < IMG_HEIGHT-2 then           -- clamp row below at the bottom
                base(2) <= base(2) + IMG_WIDTH;
              end if;
              state <= S_FETCH;
            end if;
          else
            w     <= w + 1;
            fc    <= w + 2;
            state <= S_FETCH;
          end if;

        when S_FINISH =>
          finish <= '1';
          state  <= S_IDLE;

      end case;
    end if;
  end process fsm;

end architecture rtl;

