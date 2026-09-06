`timescale 1ns/1ps // ns is the base unit of measurement , if you write #5 it will be 5 ns 
// ps is the precision with which the measurements will be tracked 
// a delay of #0.001 is tracked perfectly as 1 pico second but 0.0001 would be rounded off as it falls below precision limit

module pe_monitor(
    input logic clk, 
    input logic rst_n,
    pe_port_if.tb port // connect to the pe modport we wrote
);

import noc_pkg::*;

int unsigned ejected_count;
flit_t ejected_flits[$]; // queue for cross checking later 
// this syntax creates a dynamically sized 1D array 
// push back appends new flit at end , pop_front ejects the flit at the front

// Ejection side - log what comes out 
always_ff @( posedge clk ) begin 
    if(!rst_n) begin 
        ejected_count <= '0;
    end
    else begin 
        if(port.tb_cb.ej_valid) begin 
            ejected_count <= ejected_count + 1;
            ejected_flits.push_back(port.tb_cb.ej_flit);
            $display("[%0t] PE received flit: Dest = (%0d,%0d), Data = %h", $time, port.tb_cb.ej_flit.X_dest, port.tb_cb.ej_flit.Y_dest, port.tb_cb.ej_flit.data);
        end
    end
end
    
endmodule