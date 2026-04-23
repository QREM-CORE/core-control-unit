package core_ctrl_pkg;
    // Top-level operation modes
    typedef enum logic [1:0] {
        OP_IDLE   = 2'b00,
        OP_KEYGEN = 2'b01,
        OP_ENCAPS = 2'b10,
        OP_DECAPS = 2'b11
    } op_mode_t;
endpackage
