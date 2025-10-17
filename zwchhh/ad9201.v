module ad9201 (
    input  clk_10mhz,
    input  clk,          // 50MHz时钟（用于计时、滤波、波形判断）
    input  rst_n,  
    input  [9:0] adc_data,   
    output wire adc_clk,   
    output reg adc_select,  
    output reg sing_a, 
    output reg sing_b,
    output reg rect_wave_det  // 新增：矩形波判断结果（1=是，0=否）
);

parameter clk_FREQ = 50_000_000;  // 主时钟频率（50MHz）
parameter ADC_CLK_FREQ = 20;      // 原ADC时钟频率参数（保留）
parameter CLK_DIV = clk_FREQ / (2 * ADC_CLK_FREQ);  // 原分频参数（保留）

// 滤波参数（针对1-20KHz优化）
parameter FILTER_WINDOW = 32;  // 32点移动平均
parameter LOG2_WINDOW = 5;     // 2^5=32，移位除法用

// 动态中点相关参数（1秒统计极值）
parameter SEC_COUNT_MAX = clk_FREQ - 1;  // 1秒计时周期（50MHz→50_000_000-1）

// -------------------------- 新增：矩形波判断核心参数 --------------------------
parameter DETECT_PERIOD = clk_FREQ / 2;  // 0.5秒计时周期（50MHz→25_000_000）
parameter TRANSITION_THRESH_PERCENT = 30;  // 过渡区为dyn_adc_mid±30%
parameter RECT_MAX_TRANSITION_RATIO = 5;   // 矩形波允许的最大过渡区占比（5%）
parameter SAMPLE_BIT_WIDTH = 25;  // 样本计数位宽（2^25 > 25_000_000）
// -------------------------------------------------------------------------------

// -------------------------- 动态中点相关寄存器（保留） --------------------------
reg [31:0] sec_cnt;          // 1秒计时计数器
reg [9:0] max_i;             // 过去1秒i_data最大值
reg [9:0] min_i;             // 过去1秒i_data最小值
reg [9:0] dyn_adc_mid;       // 动态中点 = (max_i + min_i)/2
// -------------------------------------------------------------------------------

// -------------------------- 新增：矩形波判断寄存器 --------------------------
reg [31:0] detect_cnt;          // 0.5秒计时计数器
reg [SAMPLE_BIT_WIDTH-1:0] total_samples;  // 总样本数
reg [SAMPLE_BIT_WIDTH-1:0] transition_samples;  // 过渡区样本数
reg [9:0] transition_low;       // 过渡区下限（dyn_adc_mid - 30%）
reg [9:0] transition_high;      // 过渡区上限（dyn_adc_mid + 30%）
// -------------------------------------------------------------------------------

// 内部功能寄存器（保留原逻辑）
reg [7:0] clk_counter;        
reg adc_clk_reg;              
reg [9:0] data_latch;         
reg capture_i;                
reg capture_q;                
reg [2:0] pipeline_counter;   
reg [9:0] i_data_adc;         
reg [9:0] q_data_adc;         

// 跨时钟域同步寄存器（保留原逻辑）
reg [9:0] adc_data_a_sync1, adc_data_a_sync2;  // I数据同步
reg [9:0] adc_data_b_sync1, adc_data_b_sync2;  // Q数据同步

// 滤波用移位寄存器和累加器（保留原逻辑）
reg [9:0] i_filter_reg [0:FILTER_WINDOW-1];
reg [9:0] q_filter_reg [0:FILTER_WINDOW-1];
reg [9+LOG2_WINDOW:0] i_sum;  // 累加器（10+5=15位，避免溢出）
reg [9+LOG2_WINDOW:0] q_sum;  
reg [9:0] i_data;             // 滤波后I数据（矩形波判断的核心输入）
reg [9:0] q_data;             // 滤波后Q数据

// 状态机定义（保留原逻辑）
localparam STATE_IDLE     = 2'b00;
localparam STATE_CAPTURE  = 2'b01;
localparam STATE_PROCESS  = 2'b10;
reg [1:0] current_state;
reg [1:0] next_state;

// 输出AD时钟（保留原逻辑：使用clk_10mhz）
assign adc_clk = clk_10mhz;

// 冗余时钟生成逻辑（保留但注释，原逻辑未使用）
always @(posedge clk ) begin
    if (!rst_n) begin
        clk_counter <= 0;
        adc_clk_reg <= 0;
    end else begin
        if (clk_counter >= CLK_DIV - 1) begin
            clk_counter <= 0;
            adc_clk_reg <= ~adc_clk_reg;
        end else begin
            clk_counter <= clk_counter + 1;
        end
    end
end

// 状态机时序逻辑（保留原逻辑：基于clk_10mhz）
always @(posedge clk_10mhz , negedge rst_n) begin
    if (!rst_n) begin
        current_state <= STATE_IDLE;
    end else begin
        current_state <= next_state;
    end
end

// 状态机组合逻辑（保留原逻辑）
always @(*) begin
    case (current_state)
        STATE_IDLE:     next_state = STATE_CAPTURE;
        STATE_CAPTURE:  next_state = STATE_PROCESS;
        STATE_PROCESS:  next_state = STATE_CAPTURE;
        default:        next_state = STATE_IDLE;
    endcase
end

// 数据捕获逻辑（保留原逻辑：基于clk_10mhz捕获I/Q数据）
always @(posedge clk_10mhz , negedge rst_n) begin
    if (!rst_n) begin
        adc_select <= 1'b0;
        data_latch <= 10'b0;
        i_data_adc <= 10'b0;
        q_data_adc <= 10'b0;
        capture_i <= 1'b0;
        capture_q <= 1'b0;
        pipeline_counter <= 3'b0;
    end else begin
        case (current_state)
            STATE_IDLE: begin
                adc_select <= 1'b0;
                pipeline_counter <= 3'b0;
            end
            STATE_CAPTURE: begin
                data_latch <= adc_data;
                if (adc_select) begin
                    i_data_adc <= data_latch;  // 捕获I通道数据
                    capture_i <= 1'b1;
                end else begin
                    q_data_adc <= data_latch;  // 捕获Q通道数据
                    capture_q <= 1'b1;
                end
                adc_select <= ~adc_select;  // 交替选择I/Q通道
                if (pipeline_counter < 3'b111)
                    pipeline_counter <= pipeline_counter + 1;
            end
            STATE_PROCESS: begin
                if (pipeline_counter >= 3) begin
                    if (capture_i && capture_q) begin
                        capture_i <= 1'b0;
                        capture_q <= 1'b0;
                    end 
                end
            end
        endcase
    end
end

// I数据跨时钟域同步（clk_10mhz→clk，避免亚稳态）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_data_a_sync1 <= 10'd0;
        adc_data_a_sync2 <= 10'd0;
    end else begin
        adc_data_a_sync1 <= i_data_adc;
        adc_data_a_sync2 <= adc_data_a_sync1;
    end
end

// Q数据跨时钟域同步（同I数据）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_data_b_sync1 <= 10'd0;
        adc_data_b_sync2 <= 10'd0;
    end else begin
        adc_data_b_sync1 <= q_data_adc;
        adc_data_b_sync2 <= adc_data_b_sync1;
    end
end

// I通道移动平均滤波（32点滑动窗口）
integer filter_idx;  // 循环变量（避免always块内声明）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (filter_idx = 0; filter_idx < FILTER_WINDOW; filter_idx = filter_idx + 1) begin
            i_filter_reg[filter_idx] <= 10'd0;
        end
        i_sum <= 0;
    end else begin
        // 滑动窗口：减最旧值，加新值（避免重复累加）
        i_sum <= i_sum - i_filter_reg[FILTER_WINDOW-1] + adc_data_a_sync2;
        // 寄存器移位：新数据存入首地址，旧数据后移
        for (filter_idx = FILTER_WINDOW-1; filter_idx > 0; filter_idx = filter_idx - 1) begin
            i_filter_reg[filter_idx] <= i_filter_reg[filter_idx-1];
        end
        i_filter_reg[0] <= adc_data_a_sync2;
    end
end

// Q通道移动平均滤波（同I通道）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (filter_idx = 0; filter_idx < FILTER_WINDOW; filter_idx = filter_idx + 1) begin
            q_filter_reg[filter_idx] <= 10'd0;
        end
        q_sum <= 0;
    end else begin
        q_sum <= q_sum - q_filter_reg[FILTER_WINDOW-1] + adc_data_b_sync2;
        for (filter_idx = FILTER_WINDOW-1; filter_idx > 0; filter_idx = filter_idx - 1) begin
            q_filter_reg[filter_idx] <= q_filter_reg[filter_idx-1];
        end
        q_filter_reg[0] <= adc_data_b_sync2;
    end
end

// 输出滤波后数据（移位实现除法，避免资源消耗）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        i_data <= 10'd0;
        q_data <= 10'd0;
    end else begin
        i_data <= i_sum[9+LOG2_WINDOW : LOG2_WINDOW];  // 右移5位=÷32
        q_data <= q_sum[9+LOG2_WINDOW : LOG2_WINDOW];
    end
end

// 动态中点计算（1秒统计i_data极值，自适应幅值）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sec_cnt <= 32'd0;
        max_i <= 10'd0;         // 初始最大值：最小可能值（确保首次更新）
        min_i <= 10'd1023;      // 初始最小值：最大可能值（确保首次更新）
        dyn_adc_mid <= 10'd255; // 复位默认值（兼容原逻辑）
    end else begin
        if (sec_cnt >= SEC_COUNT_MAX) begin  // 1秒周期结束
            sec_cnt <= 32'd0;
            dyn_adc_mid <= (max_i + min_i) >> 1;  // 中点=（最大+最小）/2
            max_i <= i_data;  // 重置极值，准备下1秒统计
            min_i <= i_data;
        end else begin
            sec_cnt <= sec_cnt + 32'd1;
            // 实时更新当前1秒内的极值
            if (i_data > max_i) max_i <= i_data;
            if (i_data < min_i) min_i <= i_data;
        end
    end
end

// 原有极性判断（基于动态中点，保留）
always @(posedge clk) begin
    // A通道（I数据）极性
    if (i_data >= dyn_adc_mid + 10'd25)
        sing_a <= 1'b1;
    else if (i_data <= dyn_adc_mid - 10'd25)
        sing_a <= 1'b0;
    else
        sing_a <= sing_a;  // 中间范围保持原极性（抗噪声）
    
    // B通道（Q数据）极性
    if (q_data >= dyn_adc_mid + 10'd25)
        sing_b <= 1'b1;
    else if (q_data <= dyn_adc_mid - 10'd25)
        sing_b <= 1'b0;
    else
        sing_b <= sing_b;
end

// -------------------------- 新增：过渡区上下限计算 --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        transition_low <= 10'd0;
        transition_high <= 10'd1023;
    end else begin
        // 计算dyn_adc_mid ± 30%（整数近似）
        transition_low <= dyn_adc_mid - (dyn_adc_mid * TRANSITION_THRESH_PERCENT) / 100;
        transition_high <= dyn_adc_mid + (dyn_adc_mid * TRANSITION_THRESH_PERCENT) / 100;
    end
end

// -------------------------- 新增：0.5秒计时与矩形波判断 --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        detect_cnt <= 32'd0;
        total_samples <= 0;
        transition_samples <= 0;
        rect_wave_det <= 1'b0;
    end else begin
        if (detect_cnt < DETECT_PERIOD) begin
            detect_cnt <= detect_cnt + 32'd1;
            total_samples <= total_samples + 1'b1;  // 每时钟周期统计1个样本
            // 判断i_data是否在过渡区（dyn_adc_mid±30%）
            if (i_data > transition_low && i_data < transition_high) begin
                transition_samples <= transition_samples + 1'b1;
            end
        end else begin
            // 0.5秒计时结束，计算过渡区占比
            if (total_samples != 0) begin  // 避免除零错误
                // 过渡区占比 = (transition_samples * 100) / total_samples
                if ((transition_samples * 100) / total_samples < RECT_MAX_TRANSITION_RATIO) begin
                    rect_wave_det <= 1'b1;  // 过渡区占比<5% → 矩形波
                end else begin
                    rect_wave_det <= 1'b0;  // 过渡区占比≥5% → 非矩形波
                end
            end else begin
                rect_wave_det <= 1'b0;  // 无有效样本时默认非矩形波
            end
            // 重置计数器和样本数，开始下一轮0.5秒检测
            detect_cnt <= 32'd0;
            total_samples <= 0;
            transition_samples <= 0;
        end
    end
end

endmodule