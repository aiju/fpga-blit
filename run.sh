verilator $* -CFLAGS "`sdl2-config --cflags`" -LDFLAGS "`sdl2-config --libs`" --cc blit.v fx68k/*.sv -Wno-fatal --exe sim.cc && make -j -C obj_dir -f Vtop.mk && obj_dir/Vtop
