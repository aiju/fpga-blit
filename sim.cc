#include <list>
#include <unistd.h>
#include <stdlib.h>
#include <SDL.h>
#include <err.h>
#include <pthread.h>
#include "Vtop.h"
#if VM_TRACE
#include "verilated_vcd_c.h"
#else
#include "verilated.h"
#endif

std::list<char> kbd;
std::list<char> uart_in;
std::list<char> uart_out;
Vtop *tb;
pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;

void putkbd(char c)
{
	kbd.push_back(c);
	if(!tb->kbd_in_valid){
		tb->kbd_in_valid = 1;
		tb->kbd_in_data = c;
	}
}

void *recv_thread(void *)
{
	char c;
	
	for(;;){
		if(read(0, &c, 1) <= 0)
			return NULL;
		pthread_mutex_lock(&uart_mutex);
		uart_in.push_back(c);
		pthread_mutex_unlock(&uart_mutex);
	}
}

int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	tb = new Vtop;
	
	if(SDL_Init(SDL_INIT_VIDEO) < 0)
		errx(1, "SDL_Init failed");
	SDL_Window *window;
	SDL_Renderer *renderer;
	if(SDL_CreateWindowAndRenderer(800, 1024, 0, &window, &renderer) < 0)
		errx(1, "SDL_CreateWindowAndRenderer failed");
	SDL_Surface *screen = SDL_GetWindowSurface(window);
	SDL_RenderClear(renderer);
	SDL_RenderPresent(renderer);
	
	pthread_t recv_thread_pid;
	pthread_create(&recv_thread_pid, NULL, recv_thread, NULL);

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
				case SDL_QUIT: return 0;
				}
			}
		if(!tb->clk){
			if(tb->pixel_valid){
				int c = tb->pixel ? (toggle ? 200 : 255) : (toggle ? 55 : 0);
				SDL_SetRenderDrawColor(renderer, c, c, c, 255);
				SDL_RenderDrawPoint(renderer, pixx, pixy);
				if(pixx == 799){
					pixx = 0;
					SDL_RenderPresent(renderer);
					if(pixy == 1023){
						pixy = 0;
						toggle = !toggle;
					}else
						pixy++;
				}else
					pixx++;
			}
			pthread_mutex_lock(&uart_mutex);
			if(tb->uart_in_valid && tb->uart_in_ready){
				uart_in.pop_front();
				tb->uart_in_valid = 0;
			}
			if(uart_in.size() > 0){
				tb->uart_in_valid = 1;
				tb->uart_in_data = uart_in.front();
			}
			pthread_mutex_unlock(&uart_mutex);
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
