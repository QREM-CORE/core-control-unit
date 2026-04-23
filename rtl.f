# 1. Submodules (i.e. -f lib/keccak-fips202-sv/rtl.f)
-f lib/common-rtl/rtl.f

# 2. Local Packages (i.e., rtl/my_pkg.sv)
rtl/core_ctrl_pkg.sv

# 3. Local RTL (i.e., rtl/transcoder_unit.sv)
rtl/main_fsm.sv
rtl/host_if.sv

# 4. Top Level
rtl/core_control_unit.sv
