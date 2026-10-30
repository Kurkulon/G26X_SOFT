module top (input wire clk, rxd, mantx, manen, output logic mantx1, mantx2, manrx, output reg testclk);
//module top (input wire mantx, manen, output logic testclk);

reg [0:7] counter;
reg prevrxd;

always_ff @(posedge clk) begin

	if (rxd != prevrxd) begin

		prevrxd <= rxd;
		counter <= 0;

	end else if (counter == 149) begin

		testclk <= prevrxd;
		//counter <= 0;

	end else begin
		counter <= counter + 1;
	end

	manrx = testclk & (~manen);
	mantx1 = (mantx) & manen;
	mantx2 = (~mantx) & manen;

end




endmodule
