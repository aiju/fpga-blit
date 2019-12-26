#include <list>
#include <unistd.h>
#include <stdlib.h>
#include <SDL.h>
#include <err.h>
#include "Vblit.h"
#if VM_TRACE
#include "verilated_vcd_c.h"
#else
#include "verilated.h"
#endif

std::list<char> kbd;
std::list<char> uart;
Vblit *tb;

void putuart(char c)
{
	uart.push_back(c);
	if(!tb->uart_in_valid){
		tb->uart_in_valid = 1;
		tb->uart_in_data = c;
	}
}

void putkbd(char c)
{
	kbd.push_back(c);
	if(!tb->kbd_in_valid){
		tb->kbd_in_valid = 1;
		tb->kbd_in_data = c;
	}
}

int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	tb = new Vblit;
	
	if(SDL_Init(SDL_INIT_VIDEO) < 0)
		errx(1, "SDL_Init failed");
	SDL_Window *window;
	SDL_Renderer *renderer;
	if(SDL_CreateWindowAndRenderer(800, 1024, 0, &window, &renderer) < 0)
		errx(1, "SDL_CreateWindowAndRenderer failed");
	SDL_Surface *screen = SDL_GetWindowSurface(window);
	SDL_RenderClear(renderer);
	SDL_RenderPresent(renderer);	

#if VM_TRACE
	Verilated::traceEverOn(true);
	VerilatedVcdC *tfp = new VerilatedVcdC;
	tb->trace(tfp, 99);
	tfp->open("simx.vcd");
#endif

	int pixx = 0, pixy = 0, toggle = 1;
	
	tb->uart_out_ready = 1;
	tb->kbd_out_ready = 1;
	
	for(int t = 0; !Verilated::gotFinish(); t++) {
		SDL_Event ev;
		if((t & 0xff) == 0)
			while(SDL_PollEvent(&ev)){
				switch(ev.type){
				case SDL_TEXTINPUT: {
					for(char *p = ev.text.text; *p != 0; p++)
						putkbd(*p);
					break;
				}
				case SDL_QUIT: return 0;
				}
			}
		if(!tb->clk){
			if(tb->pixel_valid){
				for(int i = 0; i < 16; i++){
					int c = (tb->pixel_data & 1<<15-i) ? (toggle ? 200 : 255) : (toggle ? 55 : 0);
					SDL_SetRenderDrawColor(renderer, c, c, c, 255);
					SDL_RenderDrawPoint(renderer, pixx, pixy);
					pixx++;
				}
				if(pixx == 800){
					pixx = 0;
					SDL_RenderPresent(renderer);
					if(pixy == 1023){
						pixy = 0;
						toggle = !toggle;
					}else
						pixy++;
				}
			}
			if(tb->uart_in_valid && tb->uart_in_ready){
				uart.pop_front();
				if(uart.size() > 0)
					tb->uart_in_data = uart.front();
				else
					tb->uart_in_valid = 0;
			}
			if(tb->uart_out_valid)
				putuart(tb->uart_out_data);
			if(tb->kbd_in_valid && tb->kbd_in_ready){
				kbd.pop_front();
				if(kbd.size() > 0)
					tb->kbd_in_data = kbd.front();
				else
					tb->kbd_in_valid = 0;
			}
		}
		tb->clk = !tb->clk;
		tb->eval();
#if VM_TRACE
		tfp->dump(t);
#endif
	}
#if VM_TRACE
	tfp->close();
#endif
	
	return 0;
}
