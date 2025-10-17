module zwchhh(
		input					clk,
		input					rst_n,
		input  		[9:0] adc_data,
		output		 		adc_clk,   
		output 		 		adc_select, 
		output		[7:0]	duan,
		output		[2:0]	wei,
		output				led
);
wire				clk_10mhz;
wire				clk_1khz;
wire				clk_1hz;



wire		[15:0]		fre;
wire		[11:0]		phas;

control_div m1(
		.clk					(clk),
		.rst_n				(rst_n),
		.clk_10mhz			(clk_10mhz),
		.clk_1khz			(clk_1khz),
		.clk_1hz				(clk_1hz)
);

fre m2(
		.clk					(clk),
		.rst_n				(rst_n),
		.sig_A				(sig_A),
		.fre					(fre)
);

phas m3(
		.clk					(clk),
		.rst_n				(rst_n),
		.sig_A				(sig_A),
		.sig_B				(sig_B),
		.rect_wave_det		(rect_wave_det),
		.fre					(fre),
		.phas					(phas)
);

display m4(
		.clk_1khz			(clk_1khz),
		.rst_n				(rst_n),
		.fre					(fre),
		.phas					(phas),
		.duan					(duan),
		.wei					(wei)
);

alarm m5(
	.clk_1hz					(clk_1hz),
	.rst_n					(rst_n),
	.fre						(fre),	
	.led						(led)
);

ad9201 m6(
   .clk_10mhz(clk_10mhz),
	.clk(clk),      
	.rst_n(rst_n),   
	.adc_data(adc_data),   
	.adc_clk(adc_clk),     
	.adc_select(adc_select),  
	.sing_a(sig_A),  
	.sing_b(sig_B),
	.rect_wave_det(rect_wave_det)
);

endmodule
