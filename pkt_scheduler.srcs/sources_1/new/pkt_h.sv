package pkt_h;

typedef struct packed {
    // logic [31:0]    size;*
    // logic [63:0]     TimeStamp;*
    logic [31:0]    key;
    // logic           valid;
} pkHeadInfo;

typedef struct packed {
    logic [31:0] sIP; 
    logic [31:0] dIP; 
    logic [15:0] sPort; 
    logic [15:0] dPort; 
    logic [5:0]  NoF;
} QuadSet;

typedef struct packed {
    logic               valid;
    logic [15:0]        NoF;
    logic [3:0]         MatchFail;
    pkHeadInfo          Info;
} Ringslot;


endpackage
