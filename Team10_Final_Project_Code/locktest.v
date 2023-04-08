`timescale 1ns / 1ps
//------------------------------------------------------------------------------
module FinalProj(
    output wire [6:0] display,
    output wire [3:0] digit,
    inout wire PS2_DATA,
    inout wire PS2_CLK,
    input wire rst,
    input wire clk,
    output wire unlock,
    output pmod_1,	
	output pmod_2,	
	output pmod_4,	
	output [2:0] led_unlock,
	output checksignal
    );
    
    parameter [8:0] LEFT_SHIFT_CODES  = 9'b0_0001_0010;
    parameter [8:0] RIGHT_SHIFT_CODES = 9'b0_0101_1001;
    parameter [8:0] KEY_CODES_00 = 9'b0_0100_0101; // 0 => 45
    parameter [8:0] KEY_CODES_01 = 9'b0_0001_0110; // 1 => 16
    parameter [8:0] KEY_CODES_02 = 9'b0_0001_1110; // 2 => 1E
    parameter [8:0] KEY_CODES_03 = 9'b0_0010_0110; // 3 => 26
    parameter [8:0] KEY_CODES_04 = 9'b0_0010_0101; // 4 => 25
    parameter [8:0] KEY_CODES_05 = 9'b0_0010_1110; // 5 => 2E
    parameter [8:0] KEY_CODES_06 = 9'b0_0011_0110; // 6 => 36
    parameter [8:0] KEY_CODES_07 = 9'b0_0011_1101; // 7 => 3D
    parameter [8:0] KEY_CODES_08 = 9'b0_0011_1110; // 8 => 3E
    parameter [8:0] KEY_CODES_09 = 9'b0_0100_0110; // 9 => 46
        
    parameter [8:0] KEY_CODES_10 = 9'b0_0111_0000; // right_0 => 70
    parameter [8:0] KEY_CODES_11 = 9'b0_0110_1001; // right_1 => 69
    parameter [8:0] KEY_CODES_12 = 9'b0_0111_0010; // right_2 => 72
    parameter [8:0] KEY_CODES_13 = 9'b0_0111_1010; // right_3 => 7A
    parameter [8:0] KEY_CODES_14 = 9'b0_0110_1011; // right_4 => 6B
    parameter [8:0] KEY_CODES_15 = 9'b0_0111_0011; // right_5 => 73
    parameter [8:0] KEY_CODES_16 = 9'b0_0111_0100; // right_6 => 74
    parameter [8:0] KEY_CODES_17 = 9'b0_0110_1100; // right_7 => 6C
    parameter [8:0] KEY_CODES_18 = 9'b0_0111_0101; // right_8 => 75
    parameter [8:0] KEY_CODES_19 = 9'b0_0111_1101; // right_9 => 7D
    
    parameter [8:0] LEFT_ENTER_CODES = 9'b0_0101_1010; // 'ENTER'
    parameter [8:0] RIGHT_ENTER_CODES = 9'b1_0101_1010; // 'ENTER'
    parameter [8:0] SPACEBAR_CODES = 9'b0_0010_1001; // 'SPACEBAR'
    parameter [15:0] PASSCODE = 16'b0001_0010_0011_0100; // added [password setup
        
    reg [15:0] nums, next_nums, password;
    reg [3:0] key_num;
    reg [9:0] last_key;
    
    wire shift_down;
    wire [511:0] key_down;
    wire [8:0] last_change;
    wire been_ready;
    
    assign shift_down = (key_down[LEFT_SHIFT_CODES] == 1'b1 || key_down[RIGHT_SHIFT_CODES] == 1'b1) ? 1'b1 : 1'b0;
   
    //added
    reg unlockbuf;
    reg [2:0] errorcounter = 3'b000;
    reg speaker;
    reg timer = 1'b0;
    reg correct = 1'b1;
    reg [30:0] t_counter;
    wire [31:0] freq;
    reg reset;
    reg alarm_done=1'b0;
    reg close=1'b0;
    
    parameter [2:0] typing1 = 3'b000;
    parameter [2:0] typing2 = 3'b001;
    parameter [2:0] typing3 = 3'b010;
    parameter [2:0] open = 3'b011;
    parameter [2:0] locked = 3'b100;     
    parameter [2:0] error = 3'b101;  
    parameter [2:0] error2 = 3'b110;  
    reg [2:0] lock_state, next_lock_state;
    
    assign freq = (speaker) ? 32'd232 : 32'd0;
    assign pmod_2 = 1'd1;	
    assign pmod_4 = 1'd1;	
    assign led_unlock[0] = (lock_state == typing1) ? 1 : 0;
    assign led_unlock[1] = (lock_state == typing2) ? 1 : 0;
    assign led_unlock[2] = (lock_state == typing3) ? 1 : 0;
    assign checksignal = (next_nums == 16'b1010_1100_1110_1110) ? 1 : 0;
    assign unlock = (next_nums == 16'b1010_1100_1110_1110) ? 1 : 0;
    
    SevenSegment seven_seg (
        .display(display),
        .digit(digit),
        .nums(nums),
        .rst(rst),
        .clk(clk)
    );
        
    KeyboardDecoder key_de (
        .key_down(key_down),
        .last_change(last_change),
        .key_valid(been_ready),
        .PS2_DATA(PS2_DATA),
        .PS2_CLK(PS2_CLK),
        .rst(rst),
        .clk(clk)
    );
    
    PWM_gen pwm_0 ( 
        .clk(clk), 
        .reset(rst), 
        .freq(freq),
        .duty(10'd512), 
        .PWM(pmod_1)
    );
    
    always @ (posedge clk) begin
        if(rst) begin
            lock_state <= typing1;
            nums <= 16'b0;
            password<=16'b0;
        end else begin    
            lock_state <= next_lock_state;
            nums <= next_nums;
            password<=next_nums;
        end
    end
    
    always @ (posedge clk) begin
        if(lock_state == locked) begin
            if(t_counter > 31'd500000000) begin
                timer <= 1'b1;
            end else begin
                timer <= 1'b0;
                t_counter <= t_counter + 1'b1;
            end
        end
        if (alarm_done==1'b1) t_counter=31'd0;
    end

    always @ (*) begin
         case(lock_state)
                typing1 : 
                    begin
                    speaker=1'b0;
                    alarm_done=1'b1;
                    if (unlockbuf==1'b1) next_lock_state = open;
                    else begin
                        if (correct == 1'b0) next_lock_state = typing2;
                        else next_lock_state = typing1;
                    end
                    end
                typing2 :
                    begin
                    speaker=1'b0;
                    alarm_done=1'b1;
                    if (unlockbuf==1'b1) next_lock_state = open;
                    else begin
                        if (correct == 1'b0) next_lock_state = typing3;
                        else next_lock_state = typing2;
                    end
                    end
                typing3 :
                    begin
                    speaker=1'b0;
                    alarm_done=1'b1;
                    if (unlockbuf==1'b1) next_lock_state = open;
                    else begin
                        if (correct == 1'b0) next_lock_state = locked;
                        else next_lock_state = typing3;
                    end
                    end
                open : 
                    begin
                        speaker=1'b0;
                        if(password == 16'b0) begin
                            next_lock_state = typing1;
                            close = 1;
                        end
                        else next_lock_state = open;
                    end
                locked:
                    begin
                        speaker=1'b1;
                        if(timer==1'b1) next_lock_state = typing1;
                        else next_lock_state = locked;
                    end
                error :
                    begin
                        speaker = 1'b0;
                        if(timer == 1'b1) next_lock_state = typing2;
                        else next_lock_state = error;
                    end
                error2 :
                    begin
                        speaker = 1'b0;
                        if(timer == 1'b1) next_lock_state = typing3;
                        else next_lock_state = error2;
                    end
                default : begin
                    next_lock_state = typing1;
                end
         endcase
    end
    
    always @ (*) begin
            if (close==1'b1) unlockbuf=1'b0;
            else unlockbuf = unlockbuf;
            correct=1'b1;
            if (timer==1'b1) next_nums=16'd0;
            else next_nums = nums;
            if (lock_state!=locked) begin
                if (been_ready && key_down[last_change] == 1'b1) begin
                    if(key_num == 4'b1100) begin
                        next_nums = 16'b0;
                    end else begin
                        if(key_num == 4'b1010 || key_num == 4'b1011) begin
                            if(password == PASSCODE) begin
                                unlockbuf = 1;
                                next_nums = 16'b1010_1100_1110_1110;
                            end else begin
                                correct = 0;
                                unlockbuf = 0;
                                next_nums = 16'b1111_1011_1101_1101;
                            end
                        end else begin
                            if ((key_num != 4'b1111) && next_nums < 16'b0001_0000_0000_0000) begin
                                next_nums = {nums[11:0], key_num};
                            end else next_nums = next_nums;
                        end
                    end
                end else next_nums = next_nums;
        end
    end
    
    always @ (*) begin
        case (last_change)
            KEY_CODES_00 : key_num = 4'b0000;
            KEY_CODES_01 : key_num = 4'b0001;
            KEY_CODES_02 : key_num = 4'b0010;
            KEY_CODES_03 : key_num = 4'b0011;
            KEY_CODES_04 : key_num = 4'b0100;
            KEY_CODES_05 : key_num = 4'b0101;
            KEY_CODES_06 : key_num = 4'b0110;
            KEY_CODES_07 : key_num = 4'b0111;
            KEY_CODES_08 : key_num = 4'b1000;
            KEY_CODES_09 : key_num = 4'b1001;
            KEY_CODES_10 : key_num = 4'b0000;
            KEY_CODES_11 : key_num = 4'b0001;
            KEY_CODES_12 : key_num = 4'b0010;
            KEY_CODES_13 : key_num = 4'b0011;
            KEY_CODES_14 : key_num = 4'b0100;
            KEY_CODES_15 : key_num = 4'b0101;
            KEY_CODES_16 : key_num = 4'b0110;
            KEY_CODES_17 : key_num = 4'b0111;
            KEY_CODES_18 : key_num = 4'b1000;
            KEY_CODES_19 : key_num = 4'b1001;
            LEFT_ENTER_CODES : key_num = 4'b1010;
            RIGHT_ENTER_CODES : key_num = 4'b1011;
            SPACEBAR_CODES : key_num = 4'b1100;
            default      : key_num = 4'b1111;
        endcase
    end
    
endmodule
//--------------------------------------------------------------------
module KeyboardDecoder(
    output reg [511:0] key_down,
    output wire [8:0] last_change,
    output reg key_valid,
    inout wire PS2_DATA,
    inout wire PS2_CLK,
    input wire rst,
    input wire clk
    );

    parameter [1:0] INIT			= 2'b00;
    parameter [1:0] WAIT_FOR_SIGNAL = 2'b01;
    parameter [1:0] GET_SIGNAL_DOWN = 2'b10;
    parameter [1:0] WAIT_RELEASE    = 2'b11;
    
    parameter [7:0] IS_INIT			= 8'hAA;
    parameter [7:0] IS_EXTEND		= 8'hE0;
    parameter [7:0] IS_BREAK		= 8'hF0;
    
    reg [9:0] key, next_key;		// key = {been_extend, been_break, key_in}
    reg [1:0] state, next_state;
    reg been_ready, been_extend, been_break;
    reg next_been_ready, next_been_extend, next_been_break;
    
    wire [7:0] key_in;
    wire is_extend;
    wire is_break;
    wire valid;
    wire err;
    
    wire [511:0] key_decode = 1 << last_change;
    assign last_change = {key[9], key[7:0]};
        
    KeyboardCtrl_0 inst (
        .key_in(key_in),
        .is_extend(is_extend),
        .is_break(is_break),
        .valid(valid),
        .err(err),
        .PS2_DATA(PS2_DATA),
        .PS2_CLK(PS2_CLK),
        .rst(rst),
        .clk(clk)
    );
    
    OnePulse op (
    .single_pulse(pulse_been_ready),
    .signal(been_ready),
    .clk(clk)
    );
    
    always @ (posedge clk, posedge rst) begin
        if (rst) begin
            state <= INIT;
            been_ready  <= 1'b0;
            been_extend <= 1'b0;
            been_break  <= 1'b0;
            key <= 10'b0_0_0000_0000;
        end else begin
            state <= next_state;
            been_ready  <= next_been_ready;
            been_extend <= next_been_extend;
            been_break  <= next_been_break;
            key <= next_key;
        end
    end
    
    always @ (*) begin
        case (state)
            INIT:            next_state = (key_in == IS_INIT) ? WAIT_FOR_SIGNAL : INIT;
            WAIT_FOR_SIGNAL: next_state = (valid == 1'b0) ? WAIT_FOR_SIGNAL : GET_SIGNAL_DOWN;
            GET_SIGNAL_DOWN: next_state = WAIT_RELEASE;
            WAIT_RELEASE:    next_state = (valid == 1'b1) ? WAIT_RELEASE : WAIT_FOR_SIGNAL;
            default:         next_state = INIT;
        endcase
    end
    always @ (*) begin
        next_been_ready = been_ready;
        case (state)
            INIT:            next_been_ready = (key_in == IS_INIT) ? 1'b0 : next_been_ready;
            WAIT_FOR_SIGNAL: next_been_ready = (valid == 1'b0) ? 1'b0 : next_been_ready;
            GET_SIGNAL_DOWN: next_been_ready = 1'b1;
            WAIT_RELEASE:    next_been_ready = next_been_ready;
            default:         next_been_ready = 1'b0;
        endcase
    end
    always @ (*) begin
        next_been_extend = (is_extend) ? 1'b1 : been_extend;
        case (state)
            INIT:            next_been_extend = (key_in == IS_INIT) ? 1'b0 : next_been_extend;
            WAIT_FOR_SIGNAL: next_been_extend = next_been_extend;
            GET_SIGNAL_DOWN: next_been_extend = next_been_extend;
            WAIT_RELEASE:    next_been_extend = (valid == 1'b1) ? next_been_extend : 1'b0;
            default:         next_been_extend = 1'b0;
        endcase
    end
    always @ (*) begin
        next_been_break = (is_break) ? 1'b1 : been_break;
        case (state)
            INIT:            next_been_break = (key_in == IS_INIT) ? 1'b0 : next_been_break;
            WAIT_FOR_SIGNAL: next_been_break = next_been_break;
            GET_SIGNAL_DOWN: next_been_break = next_been_break;
            WAIT_RELEASE:    next_been_break = (valid == 1'b1) ? next_been_break : 1'b0;
            default:         next_been_break = 1'b0;
        endcase
    end
    always @ (*) begin
        next_key = key;
        case (state)
            INIT:            next_key = (key_in == IS_INIT) ? 10'b0_0_0000_0000 : next_key;
            WAIT_FOR_SIGNAL: next_key = next_key;
            GET_SIGNAL_DOWN: next_key = {been_extend, been_break, key_in};
            WAIT_RELEASE:    next_key = next_key;
            default:         next_key = 10'b0_0_0000_0000;
        endcase
    end
    
    always @ (posedge clk, posedge rst) begin
        if (rst) begin
            key_valid <= 1'b0;
            key_down <= 511'b0;
        end else if (key_decode[last_change] && pulse_been_ready) begin
            key_valid <= 1'b1;
            if (key[8] == 0) begin
                key_down <= key_down | key_decode;
            end else begin
                key_down <= key_down & (~key_decode);
            end
        end else begin
            key_valid <= 1'b0;
            key_down <= key_down;
        end
    end
    
endmodule
//-------------------------------------------------------------
module OnePulse(
    output reg single_pulse,
    input wire signal,
    input wire clk);
    
    reg signal_delay;
    
    always @(posedge clk) begin
        if(signal == 1'b1 & signal_delay == 1'b0)
            single_pulse <= 1'b1;
        else
            single_pulse <= 1'b0;
        signal_delay <= signal;
    end
endmodule
//--------------------------------------------------------------
module SevenSegment(
    output reg [6:0] display,
    output reg [3:0] digit,
    input wire [15:0] nums,
    input wire rst,
    input wire clk
    );
    
    reg [15:0] clk_divider;
    reg [3:0] display_num;
    
    always @ (posedge clk, posedge rst) begin
        if (rst) begin
            clk_divider <= 16'b0;
        end else begin
            clk_divider <= clk_divider + 16'b1;
        end
    end
    
    always @ (posedge clk, posedge rst) begin
        if (rst) begin
            display_num <= 4'b0000;
            digit <= 4'b1111;
        end else if (clk_divider == {16{1'b1}}) begin
            case (digit)
                4'b1110 : begin
                    display_num <= nums[7:4];
                    digit <= 4'b1101;
                end
                4'b1101 : begin
                    display_num <= nums[11:8];
                    digit <= 4'b1011;
                end
                4'b1011 : begin
                    display_num <= nums[15:12];
                    digit <= 4'b0111;
                end
                4'b0111 : begin
                    display_num <= nums[3:0];
                    digit <= 4'b1110;
                end
                default : begin
                    display_num <= nums[3:0];
                    digit <= 4'b1110;
                end				
            endcase
        end else begin
            display_num <= display_num;
            digit <= digit;
        end
    end
    
    always @ (*) begin
        case (display_num)
            0 : display = 7'b1000000;	//0000
            1 : display = 7'b1111001;   //0001                                                
            2 : display = 7'b0100100;   //0010                                                
            3 : display = 7'b0110000;   //0011                                             
            4 : display = 7'b0011001;   //0100                                               
            5 : display = 7'b0010010;   //0101                                               
            6 : display = 7'b0000010;   //0110
            7 : display = 7'b1111000;   //0111
            8 : display = 7'b0000000;   //1000
            9 : display = 7'b0010000;	//1001
            10 : display = 7'b0001100;  // 'P'
            11 : display = 7'b0000110;  // 'E'
            12 : display = 7'b0001000;  // 'A'
            13 : display = 7'b0101111;  // 'r'
            14 : display = 7'b0010010;  // 'S'
            default : display = 7'b0111111;
        endcase
    end
    
endmodule
//----------------------------------------------------------
module PWM_gen (
    input wire clk,
    input wire reset,
	input [31:0] freq,
    input [9:0] duty,
    output reg PWM
);

wire [31:0] count_max = 100_000_000 / freq;
wire [31:0] count_duty = count_max * duty / 1024;
reg [31:0] count;
    
always @(posedge clk, posedge reset) begin
    if (reset) begin
        count <= 0;
        PWM <= 0;
    end else if (count < count_max) begin
        count <= count + 1;
		if(count < count_duty)
            PWM <= 1;
        else
            PWM <= 0;
    end else begin
        count <= 0;
        PWM <= 0;
    end
end

endmodule