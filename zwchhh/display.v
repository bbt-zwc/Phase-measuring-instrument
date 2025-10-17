module display(
		input					 clk_1khz,
		input					 rst_n, //复位	
		input 	[15:0] fre,       //频率  10000~99999
		input	[11:0] phas,      //相位差  0~999
		output 	reg[7:0]	 duan, //数码管段选
		output 	reg[2:0]	 wei  //数码管位选
);


// 显示数据存储 (display data storage)
reg [3:0] disp_data [7:0];  // 8个数码管显示的数据 (data for 8 digital tubes)
reg [2:0] scan_cnt;          // 扫描计数器 (scanning counter)

// 七段码译码表（共阴极）(seven-segment decoder table (common cathode))
reg [7:0] SEG_TABLE [0:10];
initial begin
    SEG_TABLE[0] = 8'b0011_1111;  // 0
    SEG_TABLE[1] = 8'b0000_0110;  // 1
    SEG_TABLE[2] = 8'b0101_1011;  // 2
    SEG_TABLE[3] = 8'b0100_1111;  // 3
    SEG_TABLE[4] = 8'b0110_0110;  // 4
    SEG_TABLE[5] = 8'b0110_1101;  // 5
    SEG_TABLE[6] = 8'b0111_1101;  // 6
    SEG_TABLE[7] = 8'b0000_0111;  // 7
    SEG_TABLE[8] = 8'b0111_1111;  // 8
    SEG_TABLE[9] = 8'b0110_1111;  // 9
	SEG_TABLE[10] = 8'b0100_0000; 
end


// 扫描计数器 (scanning counter)
always @(posedge clk_1khz or negedge rst_n) begin
    if (!rst_n)
        scan_cnt <= 3'd0;
	else if (scan_cnt == 3'd7)
        scan_cnt <= 3'd0;
    else
        scan_cnt <= scan_cnt + 3'd1;
end


// 位选信号输出 - 从左到右扫描 (digit selection signal output - scan from left to right)
always @(posedge clk_1khz or negedge rst_n) begin
    if (!rst_n)
        wei <= 3'd1;  // 初始选中最左数码管（对应disp_data[0]）
    else
        wei <=  3'd1 +scan_cnt;  // 
end


reg  state;//0为小量程 1为大量程

always @(posedge clk_1khz or negedge rst_n) begin
    if (!rst_n) begin
        state <= 1'd0;
    end else begin
        // 根据频率值切换量程
        if (fre < 16'd10000) begin
            state <= 1'd0;  // 小量程：Hz单位，4位整数
        end else begin
            state <= 1'd1;  // 大量程：kHz单位，带两位小数
        end
    end
end
// 数据更新 (data update)
always @(posedge clk_1khz or negedge rst_n) begin
    if (!rst_n) begin
        disp_data[0] <= 4'd0;
        disp_data[1] <= 4'd0;
        disp_data[2] <= 4'd0;
        disp_data[3] <= 4'd0;
        disp_data[4] <= 4'd0;
        disp_data[5] <= 4'd0;
        disp_data[6] <= 4'd0;
        disp_data[7] <= 4'd0;
    end
    else begin
      if (!state)   begin
     disp_data[0] <= fre/1000%10;         // 频率千位
     disp_data[1] <= fre/100%10;         // 频率百位
     disp_data[2] <= fre/10%10;         // 频率十位
     disp_data[3] <= fre%10;         // 频率个位
      end
	else begin
     disp_data[0] <= fre/10000%10;         // 频率千位
     disp_data[1] <= fre/1000%10;         // 频率百位
     disp_data[2] <= fre/100%10;         // 频率十位
     disp_data[3] <= fre/10%10;         // 频率个位
      end
      disp_data[4] <= 4'd10;  	//分隔符
	  disp_data[5] <= (phas / 100) % 10;  // 百位（如123→1）
      disp_data[6] <= (phas / 10) % 10;   // 十位（如123→2）
      disp_data[7] <= phas % 10;          // 个位（如123→3）
    end
end

// 段选输出 (segment selection output)
always @(*) begin
    case(disp_data[scan_cnt])
        4'd0: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[0][6:0]} : SEG_TABLE[0];  // 大量程在频率个位添加小数点 (add decimal point at ones place)
        4'd1: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[1][6:0]} : SEG_TABLE[1];
        4'd2: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[2][6:0]} : SEG_TABLE[2];
        4'd3: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[3][6:0]} : SEG_TABLE[3];
        4'd4: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[4][6:0]} : SEG_TABLE[4];
        4'd5: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[5][6:0]} : SEG_TABLE[5];
        4'd6: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[6][6:0]} : SEG_TABLE[6];
        4'd7: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[7][6:0]} : SEG_TABLE[7];
        4'd8: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[8][6:0]} : SEG_TABLE[8];
        4'd9: duan = (scan_cnt == 1'd1 && state) ? {1'b1, SEG_TABLE[9][6:0]} : SEG_TABLE[9];	
		4'd10: duan = SEG_TABLE[10]; // 分隔符
        default: duan = 8'b0000_0000;
    endcase
end

endmodule