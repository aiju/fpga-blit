verilator $* -CFLAGS "`sdl2-config --cflags`" -LDFLAGS "`sdl2-config --libs`" --cc blit.v -Wno-fatal --exe sim.cc && make -j -C obj_dir -f Vblit.mk && obj_dir/Vblit
