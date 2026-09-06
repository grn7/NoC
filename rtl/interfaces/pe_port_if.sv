// Building the interface between the PE and the router 
interface pe_port_if(input logic clk);

    import noc_pkg::* ;

    // Injection signals (PE to router)
    logic inj_valid; // PE gives valid data 
    logic inj_ready; // Router is ready to receive 
    flit_t inj_flit;

    // Ejection signals (router to PE)
    logic ej_valid; // Router has valid data 
    flit_t ej_flit; 
    // No ej_ready, as we assume PE always ready to absorb an ejected flit 

    // PE modport - drives injection 
    modport pe(
        output inj_valid,
        output inj_flit,
        input inj_ready,
        input ej_valid,
        input ej_flit
    );

    // Router modport - drives ejection
    modport router(
        input inj_valid, 
        input inj_flit,
        output inj_ready,
        output ej_valid,
        output ej_flit
    );

    // Without cocotb managing simulation timeline, the testbench and the router hardware will both try to update and read signal on the exact same always_ff(posedge clk)
    // Systemverilog doesn't guarantee the order in which non blocking updates occur which can create some mismatches depending on the simulator we use 

    // Testbench only view - provides deterministic timing ; used by injector and monitor
    // A clocking block will add deterministic skew to avoid race condition 
    // Actual hardware uses signals above and testbench uses signal below 
    clocking tb_cb @(posedge clk); // cb stands for clocking block 
        output inj_valid, inj_flit;
        input inj_ready, ej_valid, ej_flit;
    endclocking

    modport tb (clocking tb_cb);

endinterface