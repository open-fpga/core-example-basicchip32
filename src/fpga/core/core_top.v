//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
//   [31:29] type
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [31:0]  cont1_key,
input   wire    [31:0]  cont2_key,
input   wire    [31:0]  cont3_key,
input   wire    [31:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_pin30 = 1'b0;      // reset or cs2, we let the hw control it by itself
assign cart_tran_pin30_dir = 1'bz;
assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
assign cart_tran_pin31 = 1'bz;      // input
assign cart_tran_pin31_dir = 1'b0;  // input

// link port is input only
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;

assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

assign sram_a = 'h0;
assign sram_dq = {16{1'bZ}};
assign sram_oe_n  = 1;
assign sram_we_n  = 1;
assign sram_ub_n  = 1;
assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;



// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        // all unmapped addresses are zero
        bridge_rd_data <= 0;
    end
    32'b000000xx_xxxxxxxx_xxxxxxxx_xxxxxxxx: begin
        bridge_rd_data <= ram1_bridge_rd_data;
    end
    32'h50000000: begin
        bridge_rd_data <= screen_border;
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end


//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked; 
    wire            status_setup_done = pll_core_locked; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire    [31:0]  dataslot_requestwrite_size;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_allcomplete;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a


// bridge data slot access

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q )

);



////////////////////////////////////////////////////////////////////////////////////////



// video generation
// ~12,288,000 hz pixel clock
//
// we want our video mode of 320x240 @ 60hz, this results in 204800 clocks per frame
// we need to add hblank and vblank times to this, so there will be a nondisplay area. 
// it can be thought of as a border around the visible area.
// to make numbers simple, we can have 400 total clocks per line, and 320 visible.
// dividing 204800 by 400 results in 512 total lines per frame, and 288 visible.
// this pixel clock is fairly high for the relatively low resolution, but that's fine.
// PLL output has a minimum output frequency anyway.


assign video_rgb_clock = clk_core_12288;
assign video_rgb_clock_90 = clk_core_12288_90deg;
assign video_rgb = vidout_rgb;
assign video_de = vidout_de;
assign video_skip = vidout_skip;
assign video_vs = vidout_vs;
assign video_hs = vidout_hs;

    localparam  VID_V_BPORCH = 'd10;
    localparam  VID_V_ACTIVE = 'd288;
    localparam  VID_V_TOTAL = 'd512;
    localparam  VID_H_BPORCH = 'd10;
    localparam  VID_H_ACTIVE = 'd320;
    localparam  VID_H_TOTAL = 'd400;
    
    reg [15:0]  frame_count;
    
    reg [9:0]   x_count;
    reg [9:0]   y_count;
    
    wire signed [9:0]  visible_x = x_count - VID_H_BPORCH;
    wire signed [9:0]  visible_y = y_count - VID_V_BPORCH;

    reg [23:0]  vidout_rgb;
    reg         vidout_de, vidout_de_1;
    reg         vidout_skip;
    reg         vidout_vs;
    reg         vidout_hs, vidout_hs_1;
    
    reg signed [9:0]   square_x = 'd135;
    reg signed [9:0]   square_y = 'd119;
    
    reg             screen_border = 1; // driven by BRIDGE clk_74a 
    wire            screen_border_s;
synch_3 s1(screen_border, screen_border_s, video_rgb_clock);

    wire            osnotify_inmenu_s; // driven by BRIDGE clk_74a in core_bridge_cmd
synch_3 s2(osnotify_inmenu, osnotify_inmenu_s, video_rgb_clock);

    wire    [31:0]  cont1_key_s;
synch_3 #(.WIDTH(32)) s22(cont1_key, cont1_key_s, video_rgb_clock);


always @(posedge video_rgb_clock or negedge reset_n) begin

    if(~reset_n) begin
    
        x_count <= 0;
        y_count <= 0;
        
    end else begin
        vidout_de <= 0;
        vidout_skip <= 0;
        vidout_vs <= 0;
        vidout_hs <= 0;
        
        vidout_hs_1 <= vidout_hs;
        vidout_de_1 <= vidout_de;
        
        // signals for the ram interface
        new_frame <= 0;
        next_line <= 0;
        
        // x and y counters
        x_count <= x_count + 1'b1;
        if(x_count == VID_H_TOTAL-1) begin
            x_count <= 0;
            
            y_count <= y_count + 1'b1;
            if(y_count == VID_V_TOTAL-1) begin
                y_count <= 0;
            end
        end
        
        // generate sync 
        if(x_count == 0 && y_count == 0) begin
            // sync signal in back porch
            // new frame
            vidout_vs <= 1;
            new_frame <= 1;
            
            if(!osnotify_inmenu_s) begin
                frame_count <= frame_count + 1'b1;
            end
        end
        
        // we want HS to occur a bit after VS, not on the same cycle
        if(x_count == 3) begin
            // sync signal in back porch
            // new line
            vidout_hs <= 1;
            
            // trigger the next_line signal 1 line ahead of the first visible line, to account for buffering
            if(y_count >= VID_V_BPORCH-1 && y_count < VID_V_ACTIVE+VID_V_BPORCH) begin
                next_line <= 1;
                linebuf_toggle <= linebuf_toggle ^ 1;
            end
        end
            
        // generate scanline buffer addressing
        // because our scanline BRAM is registered, it has an additional cycle of latency, 
        // so we must start incrementing its address a cycle early
        if(x_count >= VID_H_BPORCH-1) begin
            linebuf_rdaddr <= linebuf_rdaddr + 1'b1;
        end else begin
            linebuf_rdaddr <= 0;
        end
        
        // inactive screen areas are black
        vidout_rgb <= 24'h0;
        // generate active video
        if(x_count >= VID_H_BPORCH && x_count < VID_H_ACTIVE+VID_H_BPORCH) begin

            if(y_count >= VID_V_BPORCH && y_count < VID_V_ACTIVE+VID_V_BPORCH) begin
                // data enable. this is the active region of the line
                vidout_de <= 1;
                
                // generate the sliding XOR background
                //vidout_rgb[23:16] <= (visible_x + frame_count / 1) ^ (visible_y + frame_count/1);
                //vidout_rgb[15:8]  <= (visible_x + frame_count / 2) ^ (visible_y - frame_count/2);
                //vidout_rgb[7:0]     <= (visible_x - frame_count / 1) ^ (visible_y + 128);
                
                // convert RGB565 to RGB888
                vidout_rgb[23:16] <= {linebuf_q[15:11], linebuf_q[15:13]};
                vidout_rgb[15:8]  <= {linebuf_q[10:5], linebuf_q[10:9]};
                vidout_rgb[7:0]   <= {linebuf_q[4:0], linebuf_q[4:2]};
            
                if(screen_border_s) begin
                    // add colored borders for debugging
                    if(visible_x == 0) begin
                        vidout_rgb <= 24'hFFFFFF;
                    end else if(visible_x == VID_H_ACTIVE-1) begin
                        vidout_rgb <= 24'h00FF00;
                    end else if(visible_y == 0) begin
                        vidout_rgb <= 24'hFF0000;
                    end else if(visible_y == VID_V_ACTIVE-1) begin
                        vidout_rgb <= 24'h0000FF;
                    end
                end

            end 
        end
        
        
    end
end



    reg             next_line;
    wire            next_line_s;
synch_3 s3(next_line, next_line_s, clk_ram_controller);

    reg             new_frame;
    wire            new_frame_s;
synch_3 s4(new_frame, new_frame_s, clk_ram_controller);

    reg             display_enable;
    reg             display_enable_gated;
    wire            display_enable_s;
synch_3 s5(display_enable_gated, display_enable_s, clk_ram_controller);


    reg     [3:0]   rr_state;
    localparam      RR_STATE_0 = 'd0;
    localparam      RR_STATE_1 = 'd1;
    localparam      RR_STATE_2 = 'd2;
    localparam      RR_STATE_3 = 'd3;
    localparam      RR_STATE_4 = 'd4;
    localparam      RR_STATE_5 = 'd5;
    localparam      RR_STATE_6 = 'd6;
    localparam      RR_STATE_7 = 'd7;
    localparam      RR_STATE_8 = 'd8;
    localparam      RR_STATE_9 = 'd9;
    localparam      RR_STATE_10 = 'd10;
    
    reg     [10:0]  rr_line;
    
// fsm to handle reading ram 
//
// reset linecount on vsync, and fetch line buffers on hsync, in a pingpong buffer
always @(posedge clk_ram_controller) begin
    ram1_burst_rd <= 0;
    linebuf_wren <= 0;
    
    
    case(rr_state)
    RR_STATE_0: begin
    
        rr_state <= RR_STATE_1;
    end
    RR_STATE_1: begin
        
        if(new_frame_s) begin
            rr_line <= 'd0;
        end
        
        if(next_line_s && display_enable_s) begin
            // increment the line we will fetch next cycle
            rr_line <= rr_line + 1'b1;
            
            ram1_burst_rd <= 1'b1;
            // when displaying a contiguous buffer, we must determine the scanline
            // address with a multiplier. a better way is to fix the scanlines onto a 1024-word alignment
            // and correct the addressing as data is copied in.
            ram1_burst_addr <= rr_line * VID_H_ACTIVE; 
            ram1_burst_len <= 1024;
            ram1_burst_32bit <= 0;
            
            linebuf_wraddr <= -1;
            
            rr_state <= RR_STATE_2;
        end
    end
    RR_STATE_2: begin
        if(ram1_burst_data_valid) begin
            // ram data is valid, write into the line buffer
            linebuf_data <= ram1_burst_data;
            linebuf_wraddr <= linebuf_wraddr + 1'b1;
            linebuf_wren <= 1;
        
        end
        if(ram1_burst_data_done) begin
            rr_state <= RR_STATE_1;
        end
    
    end
    endcase

end





    reg     [31:0]  ram1_bridge_rd_data;
    
initial begin
    display_enable <= 0;
end
    
always @(posedge clk_74a) begin
    ram1_word_rd <= 0;
    ram1_word_wr <= 0;
    
    // wait til we are out of reset to start scanning out the display and hitting ram
    display_enable_gated <= display_enable & reset_n;
    
    // handle memory mapped I/O from pocket
    //
    if(bridge_wr) begin
        casex(bridge_addr[31:24])
        8'b000000xx: begin
            // 64mbyte sdram mapped at 0x0
        
            // the ram controller's word port is 32bit aligned
            ram1_word_wr <= 1;
            ram1_word_addr <= bridge_addr[25:2];
            ram1_word_data <= bridge_wr_data;
        end
        8'h50: begin
            screen_border <= bridge_wr_data;
        end
        8'h51: begin
            display_enable <= bridge_wr_data;
        end
        endcase
    end
    if(bridge_rd) begin
        casex(bridge_addr[31:24])
        8'b000000xx: begin
            // start new read
            ram1_word_rd <= 1;                  
            // convert from byte address to word address
            ram1_word_addr <= bridge_addr[25:2]; 
            // output the last value read. the requested value will be returned in time for the next read
            ram1_bridge_rd_data <= ram1_word_q; 
        end
        endcase
        
    end
    
end



//
// audio i2s silence generator
// see other examples for actual audio generation
//

assign audio_mclk = audgen_mclk;
assign audio_dac = audgen_dac;
assign audio_lrck = audgen_lrck;

// generate MCLK = 12.288mhz with fractional accumulator
    reg         [21:0]  audgen_accum = 0;
    reg                 audgen_mclk;
    parameter   [20:0]  CYCLE_48KHZ = 21'd122880 * 2;
always @(posedge clk_74a) begin
    audgen_accum <= audgen_accum + CYCLE_48KHZ;
    if(audgen_accum >= 21'd742500) begin
        audgen_mclk <= ~audgen_mclk;
        audgen_accum <= audgen_accum - 21'd742500 + CYCLE_48KHZ;
    end
end

// generate SCLK = 3.072mhz by dividing MCLK by 4
    reg [1:0]   aud_mclk_divider;
    wire        audgen_sclk = aud_mclk_divider[1] /* synthesis keep*/;
    reg         audgen_lrck_1;
always @(posedge audgen_mclk) begin
    aud_mclk_divider <= aud_mclk_divider + 1'b1;
end

// shift out audio data as I2S 
// 32 total bits per channel, but only 16 active bits at the start and then 16 dummy bits
//
    reg     [4:0]   audgen_lrck_cnt;    
    reg             audgen_lrck;
    reg             audgen_dac;
always @(negedge audgen_sclk) begin
    audgen_dac <= 1'b0;
    // 48khz * 64
    audgen_lrck_cnt <= audgen_lrck_cnt + 1'b1;
    if(audgen_lrck_cnt == 31) begin
        // switch channels
        audgen_lrck <= ~audgen_lrck;
        
    end 
end


///////////////////////////////////////////////



// note that the 12.288mhz PLL output is actually only used for video generation!
    wire    clk_core_12288;
    wire    clk_core_12288_90deg;
    wire    clk_ram_controller;
    wire    clk_ram_chip;
    wire    clk_ram_90;
    
    wire    pll_core_locked;
    
mf_pllbase mp1 (
    .refclk         ( clk_74a ),
    .rst            ( 0 ),
    
    .outclk_0       ( clk_core_12288 ),
    .outclk_1       ( clk_core_12288_90deg ),
    
    .outclk_2       ( clk_ram_controller ),
    .outclk_3       ( clk_ram_chip ),
    .outclk_4       ( clk_ram_90 ),
    
    .locked         ( pll_core_locked )
);

// clk_12288 drives the pingpong toggle for the line buffer.
// however, we need to use this toggle in the other clock domain, clk_ram_controller.
// so it's necessary to use a synchronizer to bring this into the other clock domain.
    reg             linebuf_toggle;
    wire            linebuf_toggle_s;
synch_3 s9(linebuf_toggle, linebuf_toggle_s, clk_ram_controller);

    reg     [9:0]   linebuf_rdaddr;
    wire    [10:0]  linebuf_rdaddr_fix = (linebuf_toggle ? linebuf_rdaddr : (linebuf_rdaddr + 'd1024));
    wire    [15:0]  linebuf_q;
    
    reg     [9:0]   linebuf_wraddr;
    wire    [10:0]  linebuf_wraddr_fix = (linebuf_toggle_s ? (linebuf_wraddr + 'd1024) : linebuf_wraddr);
    reg     [15:0]  linebuf_data;
    reg             linebuf_wren;

mf_linebuf  mf_linebuf_inst (
    .rdclock        ( clk_core_12288 ),
    .rdaddress      ( linebuf_rdaddr_fix ),
    .q              ( linebuf_q ),
    
    .wrclock        ( clk_ram_controller ),
    .wraddress      ( linebuf_wraddr_fix ),
    .data           ( linebuf_data ),
    .wren           ( linebuf_wren )
);


    reg             ram1_burst_rd; // must be synchronous to clk_ram
    reg     [24:0]  ram1_burst_addr;
    reg     [10:0]  ram1_burst_len;
    reg             ram1_burst_32bit;
    wire    [31:0]  ram1_burst_data;
    wire            ram1_burst_data_valid;
    wire            ram1_burst_data_done;
    
    wire            ram1_burstwr;
    wire    [24:0]  ram1_burstwr_addr;
    wire            ram1_burstwr_ready;
    wire            ram1_burstwr_strobe;
    wire    [15:0]  ram1_burstwr_data;
    wire            ram1_burstwr_done;
    
    reg             ram1_word_rd;
    reg             ram1_word_wr;
    reg     [23:0]  ram1_word_addr;
    reg     [31:0]  ram1_word_data;
    wire    [31:0]  ram1_word_q;
    wire            ram1_word_busy;

io_sdram isr0 (
    .controller_clk ( clk_ram_controller ),
    .chip_clk       ( clk_ram_chip ),
    .clk_90         ( clk_ram_90 ),
    .reset_n        ( 1'b1 ), // fsm has its own boot reset
    
    .phy_cke        ( dram_cke ),
    .phy_clk        ( dram_clk ),
    .phy_cas        ( dram_cas_n ),
    .phy_ras        ( dram_ras_n ),
    .phy_we         ( dram_we_n ),
    .phy_ba         ( dram_ba ),
    .phy_a          ( dram_a ),
    .phy_dq         ( dram_dq ),
    .phy_dqm        ( dram_dqm ),
    
    .burst_rd           ( ram1_burst_rd ),
    .burst_addr         ( ram1_burst_addr ),
    .burst_len          ( ram1_burst_len ),
    .burst_32bit        ( ram1_burst_32bit ),
    .burst_data         ( ram1_burst_data ),
    .burst_data_valid   ( ram1_burst_data_valid ),
    .burst_data_done    ( ram1_burst_data_done ),

    .burstwr        ( ram1_burstwr ),
    .burstwr_addr   ( ram1_burstwr_addr ),
    .burstwr_ready  ( ram1_burstwr_ready ),
    .burstwr_strobe ( ram1_burstwr_strobe ),
    .burstwr_data   ( ram1_burstwr_data ),
    .burstwr_done   ( ram1_burstwr_done ),
    
    .word_rd    ( ram1_word_rd ),
    .word_wr    ( ram1_word_wr ),
    .word_addr  ( ram1_word_addr ),
    .word_data  ( ram1_word_data ),
    .word_q     ( ram1_word_q ),
    .word_busy  ( ram1_word_busy )
        
);

    
endmodule
