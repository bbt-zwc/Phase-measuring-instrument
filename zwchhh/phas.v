module phas
   #(parameter    deg_180      = 8'd180,        // 180度基准
     parameter    K            = 16'd1,          // 非方波频率系数
     parameter    B            = 16'd1,          // 非方波频率偏移
     // 两级滤波参数
     parameter    WIN5000_NUM  = 8'd50,          // 总窗5000：子窗口数
     parameter    WIN5000_SIZE = 8'd100,         // 总窗5000：子窗口大小
     parameter    WIN2000_NUM  = 8'd20,          // 总窗2000：子窗口数
     parameter    WIN2000_SIZE = 8'd100,         // 总窗2000：子窗口大小
     parameter    WIN200_NUM   = 8'd2,           // 总窗200：子窗口数
     parameter    WIN200_SIZE  = 8'd100,         // 总窗200：子窗口大小
     parameter    WIN20_NUM    = 8'd2,           // 总窗20：子窗口数
     parameter    WIN20_SIZE   = 8'd10,          // 总窗20：子窗口大小
     parameter    WIN2_NUM     = 8'd2,           // 总窗2：子窗口数
     parameter    WIN2_SIZE    = 8'd1            // 总窗2：子窗口大小
   )
(
    input                   clk,
    input                   rst_n,
    input                   sig_A,
    input                   sig_B,
    input                   rect_wave_det,
    input           [15:0]  fre,
    output  reg     [11:0]  phas
);

integer filter_idx;  // 全局循环变量

// -------------------------- 1. 原始相位计算（保留） --------------------------
reg             sig_xor;
reg             old_xor;
wire            pos_xor;
reg     [31:0]  cnt_xoron, cnt_xoroff;
reg     [31:0]  cnt_xoron_reg, cnt_xoroff_reg;
reg     [11:0]  raw_phas;

always @(posedge clk) begin
    sig_xor <= sig_A ^ sig_B;
    old_xor <= sig_xor;
end
assign pos_xor = sig_xor & (~old_xor);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        cnt_xoron_reg <= 32'd0;
        cnt_xoroff_reg <= 32'd0;
        cnt_xoron <= 32'd0;
        cnt_xoroff <= 32'd0;
    end else if(pos_xor) begin
        cnt_xoron <= cnt_xoron_reg;
        cnt_xoroff <= cnt_xoroff_reg;
        cnt_xoron_reg <= 32'd0;
        cnt_xoroff_reg <= 32'd0;
    end else if(sig_xor) begin
        cnt_xoron_reg <= cnt_xoron_reg + 1'b1;
    end else begin
        cnt_xoroff_reg <= cnt_xoroff_reg + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        raw_phas <= 12'd0;
    end else if(pos_xor) begin
        raw_phas <= (cnt_xoron + cnt_xoroff == 0) ? 12'd0 : (cnt_xoron * deg_180) / (cnt_xoron + cnt_xoroff);
    end
end

// -------------------------- 2. 频率处理与窗口选择（修正窗口选择逻辑） --------------------------
reg     [15:0]  processed_fre;
reg     [12:0]  total_win;
reg     [7:0]   sub_win_num;
reg     [7:0]   sub_win_size;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        processed_fre <= 16'd0;
    end else begin
        processed_fre <= rect_wave_det ? fre : ((K * fre) + B);
    end
end

// 修正：明确窗口大小，避免因范围重叠导致窗口选择错误
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        total_win <= 13'd1;
        sub_win_num <= 8'd1;
        sub_win_size <= 8'd1;
    end else begin
        if(processed_fre >= 16'd20000) begin
            total_win <= 13'd5000;
            sub_win_num <= WIN5000_NUM;
            sub_win_size <= WIN5000_SIZE;
        end else if(processed_fre >= 16'd2000) begin  
            total_win <= 13'd2000;
            sub_win_num <= WIN2000_NUM;
            sub_win_size <= WIN2000_SIZE;
        end else if(processed_fre >= 16'd200) begin
            total_win <= 13'd200;
            sub_win_num <= WIN200_NUM;
            sub_win_size <= WIN200_SIZE;
        end else if(processed_fre >= 16'd20) begin
            total_win <= 13'd20;
            sub_win_num <= WIN20_NUM;
            sub_win_size <= WIN20_SIZE;
        end else if(processed_fre >= 16'd2) begin
            total_win <= 13'd2;
            sub_win_num <= WIN2_NUM;
            sub_win_size <= WIN2_SIZE;
        end else begin  
            total_win <= 13'd1;
            sub_win_num <= 8'd1;
            sub_win_size <= 8'd1;
        end
    end
end

// -------------------------- 3. 第一级滤波（修正子窗口均值计算条件） --------------------------
reg     [11:0]  sub_buf [0:99];
reg     [7:0]   sub_idx;
reg     [7:0]   sub_cnt;
reg     [20:0]  sub_sum;
reg     [11:0]  sub_phas;
reg             sub_valid;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sub_idx <= 8'd0;
        sub_cnt <= 8'd0;
        sub_sum <= 21'd0;
        sub_phas <= 12'd0;
        sub_valid <= 1'b0;
        for(filter_idx = 0; filter_idx < 100; filter_idx = filter_idx + 1) begin
            sub_buf[filter_idx] <= 12'd0;
        end
    end else if(pos_xor) begin
        sub_valid <= 1'b0;
        sub_buf[sub_idx] <= raw_phas;
        // 修正：启动阶段累加时，确保sum不为0（避免初始0值影响）
        if(sub_cnt < sub_win_size) begin
            sub_sum <= sub_sum + raw_phas;
            sub_cnt <= sub_cnt + 1'b1;
            // 子窗口未填满时，也临时输出均值（避免等待时间过长）
            if(sub_cnt > 0) begin  // 至少有1个有效数据就输出
                sub_phas <= sub_sum / sub_cnt;
                sub_valid <= 1'b1;
            end
        end else begin
            sub_sum <= sub_sum - sub_buf[sub_idx] + raw_phas;
            sub_phas <= sub_sum / sub_win_size;
            sub_valid <= 1'b1;
        end
        sub_idx <= (sub_idx == sub_win_size - 1) ? 8'd0 : sub_idx + 1'b1;
    end
end

// -------------------------- 4. 第二级滤波（修正最终输出条件，确保raw_phas能直接输出） --------------------------
reg     [11:0]  total_buf [0:49];
reg     [7:0]   total_idx;
reg     [7:0]   total_cnt;
reg     [20:0]  total_sum;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        total_idx <= 8'd0;
        total_cnt <= 8'd0;
        total_sum <= 21'd0;
        phas <= 12'd0;
        for(filter_idx = 0; filter_idx < 50; filter_idx = filter_idx + 1) begin
            total_buf[filter_idx] <= 12'd0;
        end
    end else begin
        // 修正1：频率<2时，直接输出raw_phas（不受滤波影响）
        if(total_win == 13'd1) begin
            phas <= raw_phas;
        end
        // 修正2：仅在需要滤波时（total_win>1）才执行两级滤波逻辑
        else if(sub_valid) begin
            total_buf[total_idx] <= sub_phas;
            if(total_cnt < sub_win_num) begin
                total_sum <= total_sum + sub_phas;
                total_cnt <= total_cnt + 1'b1;
                // 总窗口未填满时，临时输出均值（避免初始0值）
                if(total_cnt > 0) begin
                    phas <= total_sum / total_cnt;
                end
            end else begin
                total_sum <= total_sum - total_buf[total_idx] + sub_phas;
                phas <= total_sum / sub_win_num;
            end
            total_idx <= (total_idx == sub_win_num - 1) ? 8'd0 : total_idx + 1'b1;
        end
    end
end

endmodule