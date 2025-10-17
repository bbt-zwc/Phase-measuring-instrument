module fre 
   #(parameter    CLK_F = 26'd50_000_000)
	(
		input					clk,//50MHz
		input					rst_n,
		input	 				sig_A,
		output reg[15:0]	fre
);
				/*测信号频率计数*/
reg		[17:0]GATE_TIME;//仿真用2
reg				pre_cnt_a;
reg				sta_pre_cnt;
reg				gate_A;
reg				gate_clk;
reg				new_gate_A;
reg				old_gate_A;
reg				new_gate_clk;
reg				old_gate_clk;
reg				gate_reg;
reg  		[63:0]fre_temp;
reg		[15:0]gate_cnt;
reg		[31:0]cnt_gate_A;
reg		[31:0]cnt_gate_clk;
reg		[31:0]temp_cnt_gate_A;
reg		[31:0]temp_cnt_gate_clk;
wire				neg_gate_A;
wire				neg_gate_clk;

// 增强型二级滤波参数（总窗口增大，平滑效果更强）
// 总窗口 = 子组数 × 子窗口大小（比原设计增大2-3倍）
parameter SUB1_SIZE_SMALL  = 20;    // 小窗口子窗口大小（180=9×20）
parameter SUB1_NUM_SMALL   = 9;     // 小窗口子组数
parameter SUB1_SIZE_MEDIUM = 200;   // 中窗口子窗口大小（20000=200×200）
parameter SUB1_NUM_MEDIUM  = 200;     // 中窗口子组数
parameter SUB1_SIZE_LARGE  = 400;   // 大窗口子窗口大小（320000=800×400）
parameter SUB1_NUM_LARGE   = 800;   // 大窗口子组数

// 第一级滤波：子窗口缓冲区与累加器（增大位宽适配更大窗口）
reg [15:0] sub1_buf_small [SUB1_SIZE_SMALL-1:0];  // 小窗口子缓冲区
reg [15:0] sub1_buf_medium[SUB1_SIZE_MEDIUM-1:0]; // 中窗口子缓冲区
reg [15:0] sub1_buf_large [SUB1_SIZE_LARGE-1:0];  // 大窗口子缓冲区
reg [19:0] sub1_sum_small;  // 小窗口累加和（16×20=20位）
reg [22:0] sub1_sum_medium; // 中窗口累加和（16×200=22位）
reg [22:0] sub1_sum_large;  // 大窗口累加和（16×400=22位）
reg [8:0]  sub1_idx_small, sub1_idx_medium, sub1_idx_large; // 索引位宽适配
reg [8:0]  sub1_cnt_small, sub1_cnt_medium, sub1_cnt_large; // 计数位宽适配
reg [15:0] sub1_avg_small, sub1_avg_medium, sub1_avg_large; // 子窗口均值
reg        sub1_valid_small, sub1_valid_medium, sub1_valid_large; // 有效标志

// 第二级滤波：子结果缓冲区与累加器（增大平滑窗口）
reg [15:0] sub2_buf_small [SUB1_NUM_SMALL-1:0];  // 小窗口子结果缓冲区
reg [15:0] sub2_buf_medium[SUB1_NUM_MEDIUM-1:0]; // 中窗口子结果缓冲区
reg [15:0] sub2_buf_large [SUB1_NUM_LARGE-1:0];  // 大窗口子结果缓冲区
reg [19:0] sub2_sum_small;  // 小窗口总累加和（16×9=19位）
reg [21:0] sub2_sum_medium; // 中窗口总累加和（16×200=21位）
reg [22:0] sub2_sum_large;  // 大窗口总累加和（16×800=22位）
reg [8:0]  sub2_idx_small, sub2_idx_medium, sub2_idx_large; // 索引位宽适配
reg [8:0]  sub2_cnt_small, sub2_cnt_medium, sub2_cnt_large; // 计数位宽适配
reg [15:0] current_freq;    // 当前计算的频率值
reg [15:0] prev_freq;       // 上一次输出的频率（用于缓慢更新）
reg [3:0]  update_cnt;      // 更新计数器（控制输出平滑度）
integer i;                  // 全局循环变量

// 对B信号估计频率调整测量时间（保持不变）
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		pre_cnt_a <= 0;
		sta_pre_cnt <= 0;
		GATE_TIME    <= 16'd5;
	end else begin
		sta_pre_cnt <= sta_pre_cnt+1'b1;
		if(sig_A) begin
			pre_cnt_a <= pre_cnt_a+1'b1;
		end
		if(sta_pre_cnt > 50000000) begin
			sta_pre_cnt <= 50000000;
			if(pre_cnt_a < 10) begin
				GATE_TIME <= 18'd1;
			end else if(pre_cnt_a >= 10 && pre_cnt_a < 100) begin
				GATE_TIME <= 18'd10;
			end else if(pre_cnt_a >= 100 && pre_cnt_a < 1000) begin
				GATE_TIME <= 18'd500;
			end else if(pre_cnt_a >= 1000 && pre_cnt_a < 10000) begin
				GATE_TIME <= 18'd8000;
			end else if(pre_cnt_a >= 10000) begin
				GATE_TIME <= 18'd150000;
			end
		end
	end
end

// 门控信号计数器（保持不变）
always @(posedge sig_A or negedge rst_n) begin
	if(!rst_n) begin
		gate_cnt <= 16'd0;
	end else if(gate_cnt == GATE_TIME+5'd20) begin
		gate_cnt <= 16'd0;
	end else begin
		gate_cnt <= gate_cnt + 1'b1;
	end
end

// 与A信号同步的门控信号（保持不变）
always @(posedge sig_A or negedge rst_n) begin
	if(!rst_n) begin
		gate_A <= 0;
	end else if(gate_cnt < 4'd10) begin
		gate_A <= 0;
	end else if(gate_cnt < GATE_TIME+4'd10) begin
		gate_A <= 1;
	end else if(gate_cnt < GATE_TIME+5'd20) begin
		gate_A <= 0;
	end else begin
		gate_A <= 0;
	end
end

// 与clk信号同步的门控信号（保持不变）
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		gate_reg <= 0;
		gate_clk <= 0;
	end else begin
		gate_reg <= gate_A;
		gate_clk <= gate_reg;
	end
end

// 捕获与A信号同步的门控信号下降沿（保持不变）
always @(posedge sig_A or negedge rst_n) begin
	if(!rst_n) begin
		new_gate_A <= 0;
		old_gate_A <= 0;
	end else begin
		new_gate_A <= gate_A;
		old_gate_A <= new_gate_A;
	end
end
assign	neg_gate_A = old_gate_A & (~new_gate_A);

// 捕获与clk信号同步的门控信号下降沿（保持不变）
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		new_gate_clk <= 0;
		old_gate_clk <= 0;
	end else begin
		new_gate_clk <= gate_clk;
		old_gate_clk <= new_gate_clk;
	end
end
assign	neg_gate_clk = old_gate_clk & (~new_gate_clk);

// 门控时间内对A信号计数（保持不变）
always @(posedge sig_A or negedge rst_n) begin
	if(!rst_n) begin
		temp_cnt_gate_A <= 32'd0;
		cnt_gate_A		 <= 32'd0;
	end else if(gate_A) begin
		temp_cnt_gate_A <= temp_cnt_gate_A+1;
	end else if(neg_gate_A) begin
		temp_cnt_gate_A <= 32'd0;
		cnt_gate_A		 <= temp_cnt_gate_A;
	end
end

// 门控时间内对clk信号计数（保持不变）
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		temp_cnt_gate_clk <= 32'd0;
		cnt_gate_clk		<= 32'd0;
	end else if(gate_clk) begin
		temp_cnt_gate_clk <= temp_cnt_gate_clk+1;
	end else if(neg_gate_clk) begin
		temp_cnt_gate_clk <= 32'd0;
		cnt_gate_clk		<= temp_cnt_gate_clk;
	end
end

// 计算B信号频率（保持不变）
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		fre_temp <= 64'd0;
		current_freq <= 16'd0;
	end else if(gate_clk == 1'd0) begin
		fre_temp <= CLK_F * cnt_gate_A;
		current_freq <= fre_temp / cnt_gate_clk + 1;
	end
end

// -------------------------- 第一级滤波：子窗口平均（增大窗口） --------------------------
// 1. 小窗口（180=9×20）第一级
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		sub1_sum_small <= 20'd0;
		sub1_idx_small <= 9'd0;
		sub1_cnt_small <= 9'd0;
		sub1_avg_small <= 16'd0;
		sub1_valid_small <= 1'b0;
		for(i=0; i<SUB1_SIZE_SMALL; i=i+1)
			sub1_buf_small[i] <= 16'd0;
	end else if(neg_gate_clk) begin  // 每次测量完成后更新
		sub1_valid_small <= 1'b0;
		// 子窗口滑动累加（减去旧值，加新值）
		sub1_buf_small[sub1_idx_small] <= current_freq;
		if(sub1_cnt_small < SUB1_SIZE_SMALL) begin
			sub1_sum_small <= sub1_sum_small + current_freq;
			sub1_cnt_small <= sub1_cnt_small + 1'b1;
		end else begin
			sub1_sum_small <= sub1_sum_small - sub1_buf_small[sub1_idx_small] + current_freq;
		end
		// 索引循环更新
		sub1_idx_small <= (sub1_idx_small == SUB1_SIZE_SMALL-1) ? 9'd0 : sub1_idx_small + 1'b1;
		// 子窗口满，输出均值（增加启动阶段平滑）
		if(sub1_cnt_small >= SUB1_SIZE_SMALL/2) begin  // 半满即可输出，提前生效
			sub1_avg_small <= sub1_sum_small / (sub1_cnt_small + 1'b1);
			sub1_valid_small <= 1'b1;
		end
	end
end

// 2. 中窗口（1200=6×200）第一级
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		sub1_sum_medium <= 23'd0;
		sub1_idx_medium <= 9'd0;
		sub1_cnt_medium <= 9'd0;
		sub1_avg_medium <= 16'd0;
		sub1_valid_medium <= 1'b0;
		for(i=0; i<SUB1_SIZE_MEDIUM; i=i+1)
			sub1_buf_medium[i] <= 16'd0;
	end else if(neg_gate_clk) begin
		sub1_valid_medium <= 1'b0;
		sub1_buf_medium[sub1_idx_medium] <= current_freq;
		if(sub1_cnt_medium < SUB1_SIZE_MEDIUM) begin
			sub1_sum_medium <= sub1_sum_medium + current_freq;
			sub1_cnt_medium <= sub1_cnt_medium + 1'b1;
		end else begin
			sub1_sum_medium <= sub1_sum_medium - sub1_buf_medium[sub1_idx_medium] + current_freq;
		end
		sub1_idx_medium <= (sub1_idx_medium == SUB1_SIZE_MEDIUM-1) ? 9'd0 : sub1_idx_medium + 1'b1;
		if(sub1_cnt_medium >= SUB1_SIZE_MEDIUM/2) begin  // 半满输出
			sub1_avg_medium <= sub1_sum_medium / (sub1_cnt_medium + 1'b1);
			sub1_valid_medium <= 1'b1;
		end
	end
end

// 3. 大窗口（10000=50×200）第一级
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		sub1_sum_large <= 23'd0;
		sub1_idx_large <= 9'd0;
		sub1_cnt_large <= 9'd0;
		sub1_avg_large <= 16'd0;
		sub1_valid_large <= 1'b0;
		for(i=0; i<SUB1_SIZE_LARGE; i=i+1)
			sub1_buf_large[i] <= 16'd0;
	end else if(neg_gate_clk) begin
		sub1_valid_large <= 1'b0;
		sub1_buf_large[sub1_idx_large] <= current_freq;
		if(sub1_cnt_large < SUB1_SIZE_LARGE) begin
			sub1_sum_large <= sub1_sum_large + current_freq;
			sub1_cnt_large <= sub1_cnt_large + 1'b1;
		end else begin
			sub1_sum_large <= sub1_sum_large - sub1_buf_large[sub1_idx_large] + current_freq;
		end
		sub1_idx_large <= (sub1_idx_large == SUB1_SIZE_LARGE-1) ? 9'd0 : sub1_idx_large + 1'b1;
		if(sub1_cnt_large >= SUB1_SIZE_LARGE/2) begin  // 半满输出
			sub1_avg_large <= sub1_sum_large / (sub1_cnt_large + 1'b1);
			sub1_valid_large <= 1'b1;
		end
	end
end

// -------------------------- 第二级滤波：子结果平均（增强平滑） --------------------------
// 1. 小窗口（180=9×20）第二级
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		sub2_sum_small <= 20'd0;
		sub2_idx_small <= 9'd0;
		sub2_cnt_small <= 9'd0;
		for(i=0; i<SUB1_NUM_SMALL; i=i+1)
			sub2_buf_small[i] <= 16'd0;
	end else if(sub1_valid_small) begin  // 第一级输出有效时更新
		sub2_buf_small[sub2_idx_small] <= sub1_avg_small;
		if(sub2_cnt_small < SUB1_NUM_SMALL) begin
			sub2_sum_small <= sub2_sum_small + sub1_avg_small;
			sub2_cnt_small <= sub2_cnt_small + 1'b1;
		end else begin
			sub2_sum_small <= sub2_sum_small - sub2_buf_small[sub2_idx_small] + sub1_avg_small;
		end
		sub2_idx_small <= (sub2_idx_small == SUB1_NUM_SMALL-1) ? 9'd0 : sub2_idx_small + 1'b1;
	end
end

// 2. 中窗口（1200=6×200）第二级
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		sub2_sum_medium <= 22'd0;
		sub2_idx_medium <= 9'd0;
		sub2_cnt_medium <= 9'd0;
		for(i=0; i<SUB1_NUM_MEDIUM; i=i+1)
			sub2_buf_medium[i] <= 16'd0;
	end else if(sub1_valid_medium) begin
		sub2_buf_medium[sub2_idx_medium] <= sub1_avg_medium;
		if(sub2_cnt_medium < SUB1_NUM_MEDIUM) begin
			sub2_sum_medium <= sub2_sum_medium + sub1_avg_medium;
			sub2_cnt_medium <= sub2_cnt_medium + 1'b1;
		end else begin
			sub2_sum_medium <= sub2_sum_medium - sub2_buf_medium[sub2_idx_medium] + sub1_avg_medium;
		end
		sub2_idx_medium <= (sub2_idx_medium == SUB1_NUM_MEDIUM-1) ? 9'd0 : sub2_idx_medium + 1'b1;
	end
end

// 3. 大窗口（10000=50×200）第二级
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		sub2_sum_large <= 23'd0;
		sub2_idx_large <= 9'd0;
		sub2_cnt_large <= 9'd0;
		for(i=0; i<SUB1_NUM_LARGE; i=i+1)
			sub2_buf_large[i] <= 16'd0;
	end else if(sub1_valid_large) begin
		sub2_buf_large[sub2_idx_large] <= sub1_avg_large;
		if(sub2_cnt_large < SUB1_NUM_LARGE) begin
			sub2_sum_large <= sub2_sum_large + sub1_avg_large;
			sub2_cnt_large <= sub2_cnt_large + 1'b1;
		end else begin
			sub2_sum_large <= sub2_sum_large - sub2_buf_large[sub2_idx_large] + sub1_avg_large;
		end
		sub2_idx_large <= (sub2_idx_large == SUB1_NUM_LARGE-1) ? 9'd0 : sub2_idx_large + 1'b1;
	end
end

// -------------------------- 最终输出：缓慢更新机制（关键增强点） --------------------------
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		fre <= 16'd0;
		prev_freq <= 16'd0;
		update_cnt <= 4'd0;
	end else begin
		// 缓慢更新计数器：每计数到15才更新一次输出（降低响应速度，增强平滑）
		update_cnt <= update_cnt + 1'b1;
		if(update_cnt == 4'd15) begin
			update_cnt <= 4'd0;
			// 根据pre_cnt_a选择不同窗口的滤波结果
			if(pre_cnt_a < 10) begin
				// 小信号：保留一定响应速度，轻微平滑
				fre <= (current_freq + prev_freq) / 2;  // 与上一次结果平均
				prev_freq <= fre;
			end else if(pre_cnt_a < 100) begin
				// 小窗口滤波结果
				if(sub2_cnt_small >= SUB1_NUM_SMALL) begin
					fre <= (sub2_sum_small / SUB1_NUM_SMALL + prev_freq) / 2;
					prev_freq <= fre;
				end else if(sub2_cnt_small > 0) begin
					fre <= (sub2_sum_small / sub2_cnt_small + prev_freq) / 2;
					prev_freq <= fre;
				end
			end else if(pre_cnt_a < 1000) begin
				// 中窗口滤波结果
				if(sub2_cnt_medium >= SUB1_NUM_MEDIUM) begin
					fre <= (sub2_sum_medium / SUB1_NUM_MEDIUM + prev_freq) / 2;
					prev_freq <= fre;
				end else if(sub2_cnt_medium > 0) begin
					fre <= (sub2_sum_medium / sub2_cnt_medium + prev_freq) / 2;
					prev_freq <= fre;
				end
			end else begin
				// 大窗口滤波结果（最强平滑）
				if(sub2_cnt_large >= SUB1_NUM_LARGE) begin
					fre <= (sub2_sum_large / SUB1_NUM_LARGE + prev_freq) / 2;
					prev_freq <= fre;
				end else if(sub2_cnt_large > 0) begin
					fre <= (sub2_sum_large / sub2_cnt_large + prev_freq) / 2;
					prev_freq <= fre;
				end
			end
		end
	end
end

endmodule
