`timescale 1ns/1ps

module dac60501_uart_top (
    input  wire clk,          // 50 MHz
    input  wire reset_n,      // active low
    input  wire uart_rx,

    output reg  dac_sdin,
    output reg  dac_sclk,
    output reg  dac_sync_n
);

    // ============================================================
    // UART receiver: 115200 baud, 8N1, FPGA clock = 50 MHz
    // ============================================================

    localparam integer CLKS_PER_BIT = 434;
    localparam integer HALF_BIT     = 217;

    localparam [1:0] RX_IDLE  = 2'd0;
    localparam [1:0] RX_START = 2'd1;
    localparam [1:0] RX_DATA  = 2'd2;
    localparam [1:0] RX_STOP  = 2'd3;

    reg [1:0] uart_state;
    reg [8:0] uart_clk_cnt;
    reg [3:0] uart_bit_cnt;

    reg uart_rx_ff1;
    reg uart_rx_ff2;

    reg [7:0] uart_data;
    reg       uart_byte_valid;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            uart_state      <= RX_IDLE;
            uart_clk_cnt    <= 9'd0;
            uart_bit_cnt    <= 4'd0;
            uart_rx_ff1     <= 1'b1;
            uart_rx_ff2     <= 1'b1;
            uart_data       <= 8'h00;
            uart_byte_valid <= 1'b0;
        end
        else begin
            // Two-flop synchronizer.
            uart_rx_ff1 <= uart_rx;
            uart_rx_ff2 <= uart_rx_ff1;

            // Default: valid for one clock only.
            uart_byte_valid <= 1'b0;

            case (uart_state)

                RX_IDLE: begin
                    // Detect falling edge/start-bit low level.
                    if (!uart_rx_ff2) begin
                        uart_clk_cnt <= HALF_BIT - 1;
                        uart_state   <= RX_START;
                    end
                end

                RX_START: begin
                    if (uart_clk_cnt != 0) begin
                        uart_clk_cnt <= uart_clk_cnt - 1'b1;
                    end
                    else begin
                        // Confirm that the middle of start bit is still low.
                        if (!uart_rx_ff2) begin
                            // Wait a complete bit period from the start-bit
                            // centre to data-bit-0 centre.
                            uart_clk_cnt <= CLKS_PER_BIT - 1;
                            uart_bit_cnt <= 4'd0;
                            uart_state   <= RX_DATA;
                        end
                        else begin
                            // False start bit.
                            uart_state <= RX_IDLE;
                        end
                    end
                end

                RX_DATA: begin
                    if (uart_clk_cnt != 0) begin
                        uart_clk_cnt <= uart_clk_cnt - 1'b1;
                    end
                    else begin
                        // Sample one data bit at its centre.
                        uart_data[uart_bit_cnt] <= uart_rx_ff2;
                        uart_clk_cnt <= CLKS_PER_BIT - 1;

                        if (uart_bit_cnt == 4'd7) begin
                            uart_state <= RX_STOP;
                        end
                        else begin
                            uart_bit_cnt <= uart_bit_cnt + 1'b1;
                        end
                    end
                end

                RX_STOP: begin
                    if (uart_clk_cnt != 0) begin
                        uart_clk_cnt <= uart_clk_cnt - 1'b1;
                    end
                    else begin
                        // Valid 8N1 frame requires high stop bit.
                        if (uart_rx_ff2) begin
                            uart_byte_valid <= 1'b1;
                        end

                        uart_state <= RX_IDLE;
                    end
                end

                default: begin
                    uart_state <= RX_IDLE;
                end

            endcase
        end
    end


    // ============================================================
    // ASCII decimal parser
    //
    // Accepts:
    //   "5<CR>"   -> 5 mV
    //   "50<CR>"  -> 50 mV
    //   "231<CR>" -> 231 mV
    // ============================================================

    reg [9:0] target_mv;
    reg [9:0] input_mv;
    reg [1:0] digit_count;
    reg       command_ready;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            target_mv    <= 10'd0;
            input_mv     <= 10'd0;
            digit_count  <= 2'd0;
            command_ready <= 1'b0;
        end
        else begin
            command_ready <= 1'b0;

            if (uart_byte_valid) begin
                // ASCII '0' to '9'.
                if ((uart_data >= 8'h30) && (uart_data <= 8'h39)) begin
                    if (digit_count < 3) begin
                        input_mv <= input_mv * 10 + (uart_data - 8'h30);
                        digit_count <= digit_count + 1'b1;
                    end
                end

                // CR: finish command.
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
    //
    // Divider:
    //   R_TOP    = 5.36 kOhm
    //   R_BOTTOM = 1.02 kOhm
    //
    // Approximation:
    //   DAC_CODE = mv * 10 + mv / 4 + mv / 128
    // ============================================================

    wire [12:0] mv_times_10;
    wire [12:0] mv_div_4;
    wire [12:0] mv_div_128;
    wire [13:0] dac_code_calc;

    reg [12:0] dac_code;

    assign mv_times_10 = (target_mv << 3) + (target_mv << 1);
    assign mv_div_4    = target_mv >> 2;
    assign mv_div_128  = target_mv >> 7;

    assign dac_code_calc =
        mv_times_10 +
        mv_div_4 +
        mv_div_128;


    // ============================================================
    // SPI controller
    //
    // DAC60501 format:
    //   [23:16] register address
    //   [15: 0] register data
    //
    // DAC data:
    //   12-bit code occupies data[15:4]
    // ============================================================

    localparam integer SPI_DIV = 25;

    localparam [2:0] SPI_IDLE   = 3'd0;
    localparam [2:0] SPI_START  = 3'd1;
    localparam [2:0] SPI_LOW    = 3'd2;
    localparam [2:0] SPI_HIGH   = 3'd3;
    localparam [2:0] SPI_FINISH = 3'd4;

    reg [2:0] spi_state;
    reg [7:0] spi_div_counter;
    reg [5:0] spi_bit_count;
    reg [23:0] spi_shift_reg;

    reg spi_busy;
    reg spi_request;
    reg [23:0] spi_request_data;


    // ============================================================
    // Power-on initialization
    //
    // 04 00 00 : GAIN = 1x, REF-DIV = 1x
    // 03 00 00 : CONFIG
    // 08 00 00 : DAC output = 0
    // ============================================================

    localparam [2:0] INIT_WAIT   = 3'd0;
    localparam [2:0] INIT_GAIN   = 3'd1;
    localparam [2:0] INIT_CONFIG = 3'd2;
    localparam [2:0] INIT_ZERO   = 3'd3;
    localparam [2:0] INIT_DONE   = 3'd4;

    reg [2:0] init_state;
    reg [15:0] power_counter;
    reg init_done;


    // ============================================================
    // Main controller
    // ============================================================

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            dac_sdin <= 1'b0;
            dac_sclk <= 1'b0;
            dac_sync_n <= 1'b1;

            spi_state <= SPI_IDLE;
            spi_div_counter <= 8'd0;
            spi_bit_count <= 6'd0;
            spi_shift_reg <= 24'd0;

            spi_busy <= 1'b0;
            spi_request <= 1'b0;
            spi_request_data <= 24'd0;

            dac_code <= 13'd0;

            init_state <= INIT_WAIT;
            power_counter <= 16'd0;
            init_done <= 1'b0;
        end
        else begin

            // ----------------------------------------------------
            // Power-on initialization state machine.
            // ----------------------------------------------------

            case (init_state)

                INIT_WAIT: begin
                    if (power_counter < 16'd50000) begin
                        power_counter <= power_counter + 1'b1;
                    end
                    else begin
                        init_state <= INIT_GAIN;
                    end
                end

                INIT_GAIN: begin
                    if (!spi_busy && !spi_request) begin
                        spi_request_data <= 24'h040000;
                        spi_request <= 1'b1;
                        init_state <= INIT_CONFIG;
                    end
                end

                INIT_CONFIG: begin
                    if (!spi_busy && !spi_request) begin
                        spi_request_data <= 24'h030000;
                        spi_request <= 1'b1;
                        init_state <= INIT_ZERO;
                    end
                end

                INIT_ZERO: begin
                    if (!spi_busy && !spi_request) begin
                        spi_request_data <= 24'h080000;
                        spi_request <= 1'b1;
                        init_state <= INIT_DONE;
                    end
                end

                INIT_DONE: begin
                    if (!spi_busy && !spi_request) begin
                        init_done <= 1'b1;
                    end
                end

                default: begin
                    init_state <= INIT_WAIT;
                end

            endcase


            // ----------------------------------------------------
            // UART command -> DAC write request.
            // ----------------------------------------------------

            if (init_done && command_ready && !spi_busy && !spi_request) begin
                if (dac_code_calc > 14'd4095) begin
                    dac_code <= 13'd4095;
                    spi_request_data <= {8'h08, 12'hFFF, 4'h0};
                end
                else begin
                    dac_code <= dac_code_calc[12:0];
                    spi_request_data <=
                        {8'h08, dac_code_calc[11:0], 4'h0};
                end

                spi_request <= 1'b1;
            end


            // ----------------------------------------------------
            // Start SPI request.
            // ----------------------------------------------------

            if (spi_request && !spi_busy) begin
                spi_request <= 1'b0;
                spi_busy <= 1'b1;

                spi_state <= SPI_START;
                spi_div_counter <= 8'd0;
                spi_bit_count <= 6'd0;
                spi_shift_reg <= spi_request_data;

                // DAC samples SDIN on SCLK falling edge.
                // Prepare the first (bit 23) before the first SCLK edge.
                dac_sync_n <= 1'b0;
                dac_sclk <= 1'b0;
                dac_sdin <= spi_request_data[23];
            end


            // ----------------------------------------------------
            // SPI transfer.
            // ----------------------------------------------------

            if (spi_busy) begin
                case (spi_state)

                    SPI_START: begin
                        if (spi_div_counter < SPI_DIV - 1) begin
                            spi_div_counter <= spi_div_counter + 1'b1;
                        end
                        else begin
                            spi_div_counter <= 8'd0;
                            dac_sclk <= 1'b1;
                            spi_state <= SPI_HIGH;
                        end
                    end

                    SPI_HIGH: begin
                        if (spi_div_counter < SPI_DIV - 1) begin
                            spi_div_counter <= spi_div_counter + 1'b1;
                        end
                        else begin
                            spi_div_counter <= 8'd0;

                            // DAC samples the existing SDIN value here.
                            dac_sclk <= 1'b0;

                            if (spi_bit_count == 6'd23) begin
                                spi_state <= SPI_FINISH;
                            end
                            else begin
                                spi_bit_count <= spi_bit_count + 1'b1;
                                spi_shift_reg <=
                                    {spi_shift_reg[22:0], 1'b0};

                                // Do not modify SDIN in this same update.
                                spi_state <= SPI_LOW;
                            end
                        end
                    end

                    SPI_LOW: begin
                        // This occurs one 50 MHz FPGA cycle after the
                        // DAC sampling edge.  The new data is stable long
                        // before the next SCLK falling edge.
                        dac_sdin <= spi_shift_reg[23];
                        spi_state <= SPI_START;
                    end

                    SPI_FINISH: begin
                        if (spi_div_counter < SPI_DIV - 1) begin
                            spi_div_counter <= spi_div_counter + 1'b1;
                        end
                        else begin
                            spi_div_counter <= 8'd0;
                            dac_sclk <= 1'b0;
                            dac_sdin <= 1'b0;
                            dac_sync_n <= 1'b1;

                            spi_busy <= 1'b0;
                            spi_state <= SPI_IDLE;
                        end
                    end

                    default: begin
                        spi_busy <= 1'b0;
                        spi_state <= SPI_IDLE;
                        dac_sclk <= 1'b0;
                        dac_sync_n <= 1'b1;
                    end

                endcase
            end
        end
    end

endmodule
