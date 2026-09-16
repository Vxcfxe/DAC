`timescale 1ns/1ps

module dac60501_model (

    input wire sdin,
    input wire sclk,
    input wire sync_n

);

    // ============================================================
    // DAC registers
    // ============================================================

    reg [15:0] gain_reg;
    reg [15:0] config_reg;
    reg [15:0] dac_reg;

    reg [23:0] shift_reg;

    integer bit_count;
    reg frame_active;

    real vref;
    real vdac;
    real vout;

    // your resistor divider
    real R_TOP;
    real R_BOTTOM;

    initial begin

        gain_reg   = 16'h0001;
        config_reg = 16'h0000;
        dac_reg    = 16'h0000;

        shift_reg = 24'h000000;
        
		  bit_count = 0;
        frame_active = 1'b0;

        vref = 2.5;

        R_TOP    = 5360.0;
        R_BOTTOM = 1020.0;

        vdac = 0.0;
        vout = 0.0;

    end

    // A new low SYNC interval starts a new 24-bit SPI frame.
    // ============================================================

    always @(negedge sync_n) begin
        shift_reg = 24'h000000;
        bit_count = 0;
        frame_active = 1'b1;
    end

    // ============================================================
    // DAC60501 samples SDIN on falling edge of SCLK
    // ============================================================

    always @(negedge sclk) begin

        if (!sync_n) begin

            shift_reg = {shift_reg[22:0], sdin};

            bit_count = bit_count + 1;

        end

    end


    // ============================================================
    // SYNC rising edge
    //
    // DAC updates register here
    // ============================================================

    always @(posedge sync_n) begin
        if (!frame_active) begin
            // Ignore reset/initialization transitions of SYNC.
        end
        else if (bit_count == 24) begin
            $display("");
            $display("==============================================");
            $display("DAC60501 SPI FRAME");
            $display("TIME = %0t ns", $time);
            $display("FRAME = %06h", shift_reg);


            // ----------------------------------------------------
            // address
            // ----------------------------------------------------

            case (shift_reg[23:16])

                // ------------------------------------------------
                // GAIN
                // ------------------------------------------------

                8'h04: begin

                    gain_reg = shift_reg[15:0];

                    $display("REGISTER : GAIN");
                    $display("VALUE    : %04h", gain_reg);

                    if (gain_reg[0] == 1'b0)
                        $display("BUFF-GAIN = 1x");
                    else
                        $display("BUFF-GAIN = 2x");

                    if (gain_reg[8] == 1'b0)
                        $display("REF-DIV   = 1x");
                    else
                        $display("REF-DIV   = 2x");

                end


                // ------------------------------------------------
                // CONFIG
                // ------------------------------------------------

                8'h03: begin

                    config_reg = shift_reg[15:0];

                    $display("REGISTER : CONFIG");
                    $display("VALUE    : %04h", config_reg);

                end


                // ------------------------------------------------
                // DAC DATA
                // ------------------------------------------------

                8'h08: begin

                    dac_reg = shift_reg[15:0];

                    $display("REGISTER : DAC");
                    $display("RAW DATA : %04h", dac_reg);

                    calculate_voltage();

                end


                default: begin

                    $display("UNKNOWN REGISTER = %02h",
                             shift_reg[23:16]);

                end

            endcase

            $display("==============================================");
            $display("");

        end

        else begin

            $display("ERROR: SPI FRAME ONLY %0d BITS",
                     bit_count);

        end
        bit_count = 0;
        frame_active = 1'b0;
    end


    // ============================================================
    // Calculate analog output
    // ============================================================

    task calculate_voltage;

        integer dac_code;
        real gain;
        real div;

        begin

            // DAC60501 data is left aligned.
            //
            // [15:4] = 12-bit DAC code

            dac_code = dac_reg >> 4;

            // ----------------------------------------------------
            // GAIN
            // ----------------------------------------------------

            if (gain_reg[0] == 1'b0)
                gain = 1.0;
            else
                gain = 2.0;

            // ----------------------------------------------------
            // REF DIV
            // ----------------------------------------------------

            if (gain_reg[8] == 1'b0)
                div = 1.0;
            else
                div = 2.0;

            // ----------------------------------------------------
            // DAC output
            //
            // VDAC = VREF * CODE / 4096 * GAIN / DIV
            // ----------------------------------------------------

            vdac =
                vref *
                dac_code /
                4096.0 *
                gain /
                div;


            // ----------------------------------------------------
            // resistor divider
            //
            // VOUT =
            // VDAC * R_BOTTOM / (R_TOP + R_BOTTOM)
            // ----------------------------------------------------

            vout =
                vdac *
                R_BOTTOM /
                (R_TOP + R_BOTTOM);


            $display("DAC CODE = %0d", dac_code);

            $display("GAIN     = %0f", gain);

            $display("DIV      = %0f", div);

            $display("VDAC     = %0.6f V", vdac);

            $display("VOUT     = %0.6f V", vout);

            $display("VOUT     = %0.3f mV", vout * 1000.0);

        end

    endtask

endmodule
