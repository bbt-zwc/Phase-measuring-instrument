module control_div(
    input           clk,         // 输入时钟(默认50MHz)
    input           rst_n,       // 复位信号(低有效)
    output reg      clk_10mhz,   // 10MHz输出时钟
    output reg      clk_1khz,    // 1kHz输出时钟
    output reg      clk_1hz      // 1Hz输出时钟
);

// 分频参数定义(输入时钟50MHz)
parameter DIV_10MHZ_HALF1 = 2'd2;   // 10MHz半周期计数1(2个50MHz周期)
parameter DIV_10MHZ_HALF2 = 2'd3;   // 10MHz半周期计数2(3个50MHz周期)
parameter DIV_1KHZ_HALF   = 16'd25_000; // 1kHz半周期计数(25000个50MHz周期)
parameter DIV_1HZ_HALF    = 25'd25_000_000; // 1Hz半周期计数(25000000个50MHz周期)

// 计数器定义
reg [1:0] cnt_10mhz;       // 10MHz分频计数器(0-3)
reg [15:0] cnt_1khz;       // 1kHz分频计数器
reg [24:0] cnt_1hz;        // 1Hz分频计数器
reg target_10mhz;          // 10MHz交替计数目标标志(0:2, 1:3)

// 10MHz时钟生成(50MHz→10MHz，采用交替计数消除奇数分频误差)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_10mhz <= 1'b0;
        cnt_10mhz <= 2'd0;
        target_10mhz <= 1'b0; // 初始目标为2
    end else begin
        // 根据目标值计数，交替使用2和3实现平均2.5个周期的半周期
        if (target_10mhz == 1'b0) begin  // 当前目标:2
            if (cnt_10mhz >= DIV_10MHZ_HALF1 - 1'b1) begin
                clk_10mhz <= ~clk_10mhz;
                cnt_10mhz <= 2'd0;
                target_10mhz <= 1'b1;     // 切换目标为3
            end else begin
                cnt_10mhz <= cnt_10mhz + 1'b1;
            end
        end else begin  // 当前目标:3
            if (cnt_10mhz >= DIV_10MHZ_HALF2 - 1'b1) begin
                clk_10mhz <= ~clk_10mhz;
                cnt_10mhz <= 2'd0;
                target_10mhz <= 1'b0;     // 切换目标为2
            end else begin
                cnt_10mhz <= cnt_10mhz + 1'b1;
            end
        end
    end
end

// 1kHz时钟生成(50MHz→1kHz，半周期25000个50MHz时钟)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cnt_1khz <= 16'd0;
        clk_1khz <= 1'b0;
    end else if (cnt_1khz < DIV_1KHZ_HALF - 1'b1) begin
        cnt_1khz <= cnt_1khz + 1'b1;
    end else begin
        cnt_1khz <= 16'd0;
        clk_1khz <= ~clk_1khz;
    end
end

// 1Hz时钟生成(50MHz→1Hz，半周期25000000个50MHz时钟)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cnt_1hz <= 25'd0;
        clk_1hz <= 1'b0;
    end else if (cnt_1hz < DIV_1HZ_HALF - 1'b1) begin
        cnt_1hz <= cnt_1hz + 1'b1;
    end else begin
        cnt_1hz <= 25'd0;
        clk_1hz <= ~clk_1hz;
    end
end

endmodule
