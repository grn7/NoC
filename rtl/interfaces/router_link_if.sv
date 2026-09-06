// Router to router link 
// No ready signal since it is bufferless deflection routing 
// every link will either carry a flit in a cycle or no flit ; no backpressure

// modules are for processing and manipulating data whereas interfaces are only to transport the data 
// modport define strict input and output rules to prevent conflicts 

interface router_link_if;

    import noc_pkg::*;

    logic valid;
    flit_t flit;

    // modport src defines that source must output valid and flit and input of sink should be valid and flit 
    modport src(output valid, output flit);
    modport snk(input valid, input flit);
    
endinterface 