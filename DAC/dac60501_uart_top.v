`timescale 1ns/1ps

module dac60501_uart_top (
    input  wire clk,          // 50 MHz
    input  wire reset_n,      // active low, PIN_86
    input  wire uart_rx,      // PC -> FPGA, PIN_22

    output reg  uart_tx,      // FPGA -> PC, PIN_21
    output reg  dac_sdin,
    output reg  dac_sclk,
    output reg  dac_sync_n
);

    // ============================================================
    // Clock and power-on reset
    // ============================================================
    // These are explicitly 9-bit values to avoid Quartus warning 10230.
    localparam [8:0] UART_BIT_LAST   = 9'd433; // 434 clock cycles per UART bit
    localparam [8:0] UART_HALF_LAST  = 9'd216; // 217 clock cycles

    // An external reset held at 3.3 V is inactive.  This internal POR
    // guarantees a reset interval after FPGA configuration as well.
    reg [5:0] power_on_reset_count = 6'd0;
    reg       power_on_reset_done  = 1'b0;
    wire      internal_reset_n;

    assign internal_reset_n = reset_n & power_on_reset_done;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            power_on_reset_count <= 6'd0;
            power_on_reset_done  <= 1'b0;
        end
        else if (!power_on_reset_done) begin
            if (power_on_reset_count == 6'd63)
                power_on_reset_done <= 1'b1;
            else
                power_on_reset_count <= power_on_reset_count + 1'b1;
        end
    end

    // ============================================================
    // UART receiver: 115200 baud, 8N1
    // ============================================================
    localparam [1:0] RX_IDLE  = 2'd0;
    localparam [1:0] RX_START = 2'd1;
    localparam [1:0] RX_DATA  = 2'd2;
    localparam [1:0] RX_STOP  = 2'd3;

    reg [1:0] uart_state;
    reg [8:0] uart_clk_cnt;
    reg [3:0] uart_bit_cnt;
    reg       uart_rx_ff1;
    reg       uart_rx_ff2;
    reg [7:0] uart_data;
    reg       uart_byte_valid;

    always @(posedge clk or negedge internal_reset_n) begin
        if (!internal_reset_n) begin
            uart_state      <= RX_IDLE;
            uart_clk_cnt    <= 9'd0;
            uart_bit_cnt    <= 4'd0;
            uart_rx_ff1     <= 1'b1;
            uart_rx_ff2     <= 1'b1;
            uart_data       <= 8'h00;
            uart_byte_valid <= 1'b0;
        end
        else begin
            uart_rx_ff1     <= uart_rx;
            uart_rx_ff2     <= uart_rx_ff1;
            uart_byte_valid <= 1'b0;

            case (uart_state)
                RX_IDLE: begin
                    if (!uart_rx_ff2) begin
                        uart_clk_cnt <= UART_HALF_LAST;
                        uart_state   <= RX_START;
                    end
                end

                RX_START: begin
                    if (uart_clk_cnt != 0)
                        uart_clk_cnt <= uart_clk_cnt - 1'b1;
                    else if (!uart_rx_ff2) begin
                        uart_clk_cnt <= UART_BIT_LAST;
                        uart_bit_cnt <= 4'd0;
                        uart_state   <= RX_DATA;
                    end
                    else
                        uart_state <= RX_IDLE;
                end

                RX_DATA: begin
                    if (uart_clk_cnt != 0)
                        uart_clk_cnt <= uart_clk_cnt - 1'b1;
                    else begin
                        uart_data[uart_bit_cnt] <= uart_rx_ff2;
                        uart_clk_cnt <= UART_BIT_LAST;
                        if (uart_bit_cnt == 4'd7)
                            uart_state <= RX_STOP;
                        else
                            uart_bit_cnt <= uart_bit_cnt + 1'b1;
                    end
                end

                RX_STOP: begin
                    if (uart_clk_cnt != 0)
                        uart_clk_cnt <= uart_clk_cnt - 1'b1;
                    else begin
                        if (uart_rx_ff2)
                            uart_byte_valid <= 1'b1;
                        uart_state <= RX_IDLE;
                    end
                end

                default: uart_state <= RX_IDLE;
            endcase
        end
    end

    // ============================================================
    // UART transmitter: 115200 baud, 8N1
    // ============================================================
    reg       uart_tx_start;
    reg [7:0] uart_tx_data;
    reg       uart_tx_busy;
    reg [8:0] uart_tx_clk_cnt;
    reg [3:0] uart_tx_bit_cnt;
    reg [7:0] uart_tx_shift;

    always @(posedge clk or negedge internal_reset_n) begin
        if (!internal_reset_n) begin
            uart_tx            <= 1'b1;
            uart_tx_busy       <= 1'b0;
            uart_tx_clk_cnt    <= 9'd0;
            uart_tx_bit_cnt    <= 4'd0;
            uart_tx_shift      <= 8'h00;
        end
        else if (!uart_tx_busy) begin
            uart_tx <= 1'b1;
            if (uart_tx_start) begin
                uart_tx         <= 1'b0; // start bit
                uart_tx_busy    <= 1'b1;
                uart_tx_clk_cnt <= 9'd0;
                uart_tx_bit_cnt <= 4'd0;
                uart_tx_shift   <= uart_tx_data;
            end
        end
        else if (uart_tx_clk_cnt == UART_BIT_LAST) begin
            uart_tx_clk_cnt <= 9'd0;
            if (uart_tx_bit_cnt < 4'd8) begin
                uart_tx <= uart_tx_shift[uart_tx_bit_cnt];
                uart_tx_bit_cnt <= uart_tx_bit_cnt + 1'b1;
            end
            else if (uart_tx_bit_cnt == 4'd8) begin
                uart_tx <= 1'b1; // stop bit
                uart_tx_bit_cnt <= 4'd9;
            end
            else begin
                uart_tx <= 1'b1;
                uart_tx_busy <= 1'b0;
            end
        end
        else begin
            uart_tx_clk_cnt <= uart_tx_clk_cnt + 1'b1;
        end
    end

    // ============================================================
    // UART debug event FIFO
    //
    // Each event is sent as one abbreviation plus CR/LF.  This avoids
    // long messages and lets the terminal show every state clearly.
    // ============================================================
    localparam [3:0] DBG_BOOT       = 4'd0;  // B
    localparam [3:0] DBG_WAIT_DONE  = 4'd1;  // W
    localparam [3:0] DBG_INIT_GAIN  = 4'd2;  // G
    localparam [3:0] DBG_INIT_CFG   = 4'd3;  // C
    localparam [3:0] DBG_INIT_ZERO  = 4'd4;  // Z
    localparam [3:0] DBG_INIT_DONE  = 4'd5;  // I
    localparam [3:0] DBG_CMD_OK     = 4'd6;  // R
    localparam [3:0] DBG_SPI_GAIN   = 4'd7;  // g
    localparam [3:0] DBG_SPI_CFG    = 4'd8;  // c
    localparam [3:0] DBG_SPI_ZERO   = 4'd9;  // z
    localparam [3:0] DBG_SPI_DAC    = 4'd10; // d
    localparam [3:0] DBG_SPI_DONE   = 4'd11; // H
    localparam [3:0] DBG_OVERFLOW   = 4'd12; // F

    reg [3:0] debug_fifo [0:15];
    reg [3:0] debug_wr_ptr;
    reg [3:0] debug_rd_ptr;
    reg       debug_overflow;

    function [7:0] debug_code;
        input [3:0] event_id;
        begin
            case (event_id)
                DBG_BOOT:      debug_code = "B";
                DBG_WAIT_DONE: debug_code = "W";
                DBG_INIT_GAIN: debug_code = "G";
                DBG_INIT_CFG:  debug_code = "C";
                DBG_INIT_ZERO: debug_code = "Z";
                DBG_INIT_DONE: debug_code = "I";
                DBG_CMD_OK:    debug_code = "R";
                DBG_SPI_GAIN:  debug_code = "g";
                DBG_SPI_CFG:   debug_code = "c";
                DBG_SPI_ZERO:  debug_code = "z";
                DBG_SPI_DAC:   debug_code = "d";
                DBG_SPI_DONE:  debug_code = "H";
                default:       debug_code = "F";
            endcase
        end
    endfunction

    localparam [1:0] DBG_TX_IDLE = 2'd0;
    localparam [1:0] DBG_TX_CODE = 2'd1;
    localparam [1:0] DBG_TX_CR   = 2'd2;
    localparam [1:0] DBG_TX_LF   = 2'd3;

    reg [1:0] debug_tx_state;
    reg [3:0] debug_tx_event;
    reg       debug_wait_busy;

    always @(posedge clk or negedge internal_reset_n) begin
        if (!internal_reset_n) begin
            debug_rd_ptr    <= 4'd0;
            debug_tx_state  <= DBG_TX_IDLE;
            debug_tx_event  <= DBG_BOOT;
            debug_wait_busy <= 1'b0;
            uart_tx_start   <= 1'b0;
            uart_tx_data    <= 8'h00;
        end
        else begin
            uart_tx_start <= 1'b0;

            // Wait until the UART transmitter has accepted the start pulse,
            // then wait for its complete 8N1 character before the next byte.
            if (debug_wait_busy) begin
                if (uart_tx_busy)
                    debug_wait_busy <= 1'b0;
            end
            else if (!uart_tx_busy) begin
                case (debug_tx_state)
                    DBG_TX_IDLE: begin
                        if (debug_rd_ptr != debug_wr_ptr) begin
                            debug_tx_event <= debug_fifo[debug_rd_ptr];
                            debug_rd_ptr   <= debug_rd_ptr + 1'b1;
                            debug_tx_state <= DBG_TX_CODE;
                        end
                    end

                    DBG_TX_CODE: begin
                        uart_tx_data    <= debug_code(debug_tx_event);
                        uart_tx_start   <= 1'b1;
                        debug_wait_busy <= 1'b1;
                        debug_tx_state  <= DBG_TX_CR;
                    end

                    DBG_TX_CR: begin
                        uart_tx_data    <= 8'h0D;
                        uart_tx_start   <= 1'b1;
                        debug_wait_busy <= 1'b1;
                        debug_tx_state  <= DBG_TX_LF;
                    end

                    default: begin
                        uart_tx_data    <= 8'h0A;
                        uart_tx_start   <= 1'b1;
                        debug_wait_busy <= 1'b1;
                        debug_tx_state  <= DBG_TX_IDLE;
                    end
                endcase
            end
        end
    end

    // ============================================================
    // ASCII decimal parser: "5<CR>", "50<CR>", "231<CR>"
    // ============================================================
    reg [9:0] target_mv;
    reg [9:0] input_mv;
    reg [1:0] digit_count;
    reg       command_ready;

    always @(posedge clk or negedge internal_reset_n) begin
        if (!internal_reset_n) begin
            target_mv     <= 10'd0;
            input_mv      <= 10'd0;
            digit_count   <= 2'd0;
            command_ready <= 1'b0;
        end
        else begin
            command_ready <= 1'b0;
            if (uart_byte_valid) begin
                if ((uart_data >= 8'h30) && (uart_data <= 8'h39)) begin
                    if (digit_count < 3) begin
                        input_mv    <= input_mv * 10 + (uart_data - 8'h30);
                        digit_count <= digit_count + 1'b1;
                    end
                end
                else if (uart_data == 8'h0D) begin
                    if (digit_count != 0) begin
                        target_mv     <= input_mv;
                        command_ready <= 1'b1;
                    end
                    input_mv    <= 10'd0;
                    digit_count <= 2'd0;
                end
            end
        end
    end

    // ============================================================
    // mV -> 12-bit DAC code
    // ============================================================
    wire [12:0] mv_times_10  = (target_mv << 3) + (target_mv << 1);
    wire [12:0] mv_div_4     = target_mv >> 2;
    wire [12:0] mv_div_128   = target_mv >> 7;
    wire [13:0] dac_code_calc = mv_times_10 + mv_div_4 + mv_div_128;
    reg  [12:0] dac_code;

    // ============================================================
    // SPI and initialization controller
    // ============================================================
    localparam integer SPI_DIV = 25; // approximately 1 MHz SCLK

    localparam [2:0] SPI_IDLE   = 3'd0;
    localparam [2:0] SPI_START  = 3'd1;
    localparam [2:0] SPI_LOW    = 3'd2;
    localparam [2:0] SPI_HIGH   = 3'd3;
    localparam [2:0] SPI_FINISH = 3'd4;

    localparam [2:0] INIT_WAIT   = 3'd0;
    localparam [2:0] INIT_GAIN   = 3'd1;
    localparam [2:0] INIT_CONFIG = 3'd2;
    localparam [2:0] INIT_ZERO   = 3'd3;
    localparam [2:0] INIT_DONE   = 3'd4;

    reg [2:0]  spi_state;
    reg [7:0]  spi_div_counter;
    reg [5:0]  spi_bit_count;
    reg [23:0] spi_shift_reg;
    reg        spi_busy;
    reg        spi_request;
    reg [23:0] spi_request_data;
    reg [2:0]  init_state;
    reg [15:0] power_counter;
    reg        init_done;
    reg        boot_logged;

    always @(posedge clk or negedge internal_reset_n) begin
        if (!internal_reset_n) begin
            dac_sdin          <= 1'b0;
            dac_sclk          <= 1'b0;
            dac_sync_n        <= 1'b1;
            spi_state         <= SPI_IDLE;
            spi_div_counter   <= 8'd0;
            spi_bit_count     <= 6'd0;
            spi_shift_reg     <= 24'd0;
            spi_busy          <= 1'b0;
            spi_request       <= 1'b0;
            spi_request_data  <= 24'd0;
            dac_code          <= 13'd0;
            init_state        <= INIT_WAIT;
            power_counter     <= 16'd0;
            init_done         <= 1'b0;
            boot_logged       <= 1'b0;
            debug_wr_ptr      <= 4'd0;
            debug_overflow    <= 1'b0;
        end
        else begin
            // Queue BOOT once after internal reset is released.
            if (!boot_logged) begin
                debug_fifo[debug_wr_ptr] <= DBG_BOOT;
                debug_wr_ptr <= debug_wr_ptr + 1'b1;
                boot_logged <= 1'b1;
            end

            case (init_state)
                INIT_WAIT: begin
                    if (power_counter < 16'd50000)
                        power_counter <= power_counter + 1'b1;
                    else begin
                        init_state <= INIT_GAIN;
                        if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                            debug_fifo[debug_wr_ptr] <= DBG_WAIT_DONE;
                            debug_wr_ptr <= debug_wr_ptr + 1'b1;
                        end
                        else
                            debug_overflow <= 1'b1;
                    end
                end

                INIT_GAIN: begin
                    if (!spi_busy && !spi_request) begin
                        spi_request_data <= 24'h040000;
                        spi_request      <= 1'b1;
                        init_state       <= INIT_CONFIG;
                        if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                            debug_fifo[debug_wr_ptr] <= DBG_INIT_GAIN;
                            debug_wr_ptr <= debug_wr_ptr + 1'b1;
                        end
                        else
                            debug_overflow <= 1'b1;
                    end
                end

                INIT_CONFIG: begin
                    if (!spi_busy && !spi_request) begin
                        spi_request_data <= 24'h030000;
                        spi_request      <= 1'b1;
                        init_state       <= INIT_ZERO;
                        if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                            debug_fifo[debug_wr_ptr] <= DBG_INIT_CFG;
                            debug_wr_ptr <= debug_wr_ptr + 1'b1;
                        end
                        else
                            debug_overflow <= 1'b1;
                    end
                end

                INIT_ZERO: begin
                    if (!spi_busy && !spi_request) begin
                        spi_request_data <= 24'h080000;
                        spi_request      <= 1'b1;
                        init_state       <= INIT_DONE;
                        if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                            debug_fifo[debug_wr_ptr] <= DBG_INIT_ZERO;
                            debug_wr_ptr <= debug_wr_ptr + 1'b1;
                        end
                        else
                            debug_overflow <= 1'b1;
                    end
                end

                INIT_DONE: begin
                    if (!spi_busy && !spi_request && !init_done) begin
                        init_done <= 1'b1;
                        if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                            debug_fifo[debug_wr_ptr] <= DBG_INIT_DONE;
                            debug_wr_ptr <= debug_wr_ptr + 1'b1;
                        end
                        else
                            debug_overflow <= 1'b1;
                    end
                end

                default: init_state <= INIT_WAIT;
            endcase

            // A complete decimal command was received from the PC.
            if (init_done && command_ready && !spi_busy && !spi_request) begin
                if (dac_code_calc > 14'd4095) begin
                    dac_code         <= 13'd4095;
                    spi_request_data <= {8'h08, 12'hFFF, 4'h0};
                end
                else begin
                    dac_code         <= dac_code_calc[12:0];
                    spi_request_data <= {8'h08, dac_code_calc[11:0], 4'h0};
                end
                spi_request <= 1'b1;
                if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                    debug_fifo[debug_wr_ptr] <= DBG_CMD_OK;
                    debug_wr_ptr <= debug_wr_ptr + 1'b1;
                end
                else
                    debug_overflow <= 1'b1;
            end

            // Start of every SPI frame: SYNC is asserted low here.
            if (spi_request && !spi_busy) begin
                spi_request       <= 1'b0;
                spi_busy          <= 1'b1;
                spi_state         <= SPI_START;
                spi_div_counter   <= 8'd0;
                spi_bit_count     <= 6'd0;
                spi_shift_reg     <= spi_request_data;
                dac_sync_n        <= 1'b0;
                dac_sclk          <= 1'b0;
                dac_sdin          <= spi_request_data[23];

                if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                    case (spi_request_data[23:16])
                        8'h04: debug_fifo[debug_wr_ptr] <= DBG_SPI_GAIN;
                        8'h03: debug_fifo[debug_wr_ptr] <= DBG_SPI_CFG;
                        8'h08: begin
                            if (init_done)
                                debug_fifo[debug_wr_ptr] <= DBG_SPI_DAC;
                            else
                                debug_fifo[debug_wr_ptr] <= DBG_SPI_ZERO;
                        end
                        default: debug_fifo[debug_wr_ptr] <= DBG_OVERFLOW;
                    endcase
                    debug_wr_ptr <= debug_wr_ptr + 1'b1;
                end
                else
                    debug_overflow <= 1'b1;
            end

            if (spi_busy) begin
                case (spi_state)
                    SPI_START: begin
                        if (spi_div_counter < SPI_DIV - 1)
                            spi_div_counter <= spi_div_counter + 1'b1;
                        else begin
                            spi_div_counter <= 8'd0;
                            dac_sclk <= 1'b1;
                            spi_state <= SPI_HIGH;
                        end
                    end

                    SPI_HIGH: begin
                        if (spi_div_counter < SPI_DIV - 1)
                            spi_div_counter <= spi_div_counter + 1'b1;
                        else begin
                            spi_div_counter <= 8'd0;
                            dac_sclk <= 1'b0; // DAC samples SDIN here
                            if (spi_bit_count == 6'd23)
                                spi_state <= SPI_FINISH;
                            else begin
                                spi_bit_count <= spi_bit_count + 1'b1;
                                spi_shift_reg <= {spi_shift_reg[22:0], 1'b0};
                                spi_state <= SPI_LOW;
                            end
                        end
                    end

                    SPI_LOW: begin
                        // SDIN changes one FPGA clock after SCLK falling edge.
                        dac_sdin <= spi_shift_reg[23];
                        spi_state <= SPI_START;
                    end

                    SPI_FINISH: begin
                        if (spi_div_counter < SPI_DIV - 1)
                            spi_div_counter <= spi_div_counter + 1'b1;
                        else begin
                            spi_div_counter <= 8'd0;
                            dac_sclk   <= 1'b0;
                            dac_sdin   <= 1'b0;
                            dac_sync_n <= 1'b1; // end of frame: SYNC released
                            spi_busy   <= 1'b0;
                            spi_state  <= SPI_IDLE;
                            if ((debug_wr_ptr + 1'b1) != debug_rd_ptr) begin
                                debug_fifo[debug_wr_ptr] <= DBG_SPI_DONE;
                                debug_wr_ptr <= debug_wr_ptr + 1'b1;
                            end
                            else
                                debug_overflow <= 1'b1;
                        end
                    end

                    default: begin
                        spi_busy   <= 1'b0;
                        spi_state  <= SPI_IDLE;
                        dac_sclk   <= 1'b0;
                        dac_sync_n <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule
