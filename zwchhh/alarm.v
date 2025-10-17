module alarm(
	input				 clk_1hz,
	input 			 rst_n,
	input		[15:0] fre,	
	output 		reg led
);
 
always@(posedge clk_1hz or negedge rst_n)
begin
	if(!rst_n)
		led<=1'b0;
	else if(fre>16'd20_000)
		led<=~led;
	else
		led<=1'b0;
		
end 
endmodule
