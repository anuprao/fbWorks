// gcc -o fb_redirectio fb_redirectio.c

#include <stdio.h>     
#include <stdlib.h>
#include <string.h>      
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/epoll.h> 

#define MAX_EVENTS			5
#define READ_SIZE			160

#define FN_INPUT_FIFO		"INITWRAPPER_INPUT"
#define FN_INPUT_READ		FN_INPUT_FIFO
#define FN_INPUT_WRITE		FN_INPUT_FIFO

#define FN_OUTPUT_FIFO		"INITWRAPPER_OUTPUT"
#define FN_OUTPUT_READ		FN_OUTPUT_FIFO
#define FN_OUTPUT_WRITE		FN_OUTPUT_FIFO

int main()
{
	//char* const args[] = {"python3", "fake_switch_root.py", NULL}; 
	//char* const args[] = {"/bin/busybox", "switch_root", "new_root", "new_init", NULL}; 
	char* const args[] = {"/mnt/workspace/community/embedded_linux/Busybox-static/busybox_amd64", "sh", "fake_init_script.sh", NULL}; 

	int fd_output_read = -1;	
	int fd_input_write = -1;

	//

	{
		//

		int epoll_fd ;

		printf("Hello from Virtual Exec Redirect ... pid=%d!\n", getpid());  

		//

		epoll_fd = epoll_create1(0);
	
		if(epoll_fd == -1)
		{
			fprintf(stderr, "Failed to create epoll file descriptor\n");
			return 1;
		}

		{	
			// parent process because return value non-zero. 

			//
			
			fd_output_read = open(FN_OUTPUT_READ, O_RDONLY | O_NONBLOCK);
			if(0 < fd_output_read)
			{
				printf("Opened  fd_output_read = %d\n", fd_output_read);
			}
			else
			{
				perror("Could not open fd_output_read");
			}
			
			// USe RDWR since, as WRONLY can only be used if the pipe is known to be aleady opened for reading
			fd_input_write = open(FN_INPUT_WRITE, O_RDWR | O_NONBLOCK);
			if(0 < fd_input_write)
			{			
				printf("Opened  fd_input_write = %d\n", fd_input_write);
			}
			else
			{
				perror("Could not open fd_input_write");
			}			
			
			//
			{
				//

				dup2(fd_output_read, STDIN_FILENO);    // fd_output_read (the read end of pipe) 
				close(fd_output_read);                 // file descriptor no longer needed in child since stdin is a copy

				//close(fd_output_read);                 // file descriptor unused in child

				//

				dup2(fd_input_write, STDOUT_FILENO);   // fd_input_write (the write end of pipe) 
				close(fd_input_write);                 // file descriptor no longer needed in child since stdout is a copy

				//close(fd_input_write);                 // file descriptor unused in child	

				//					

				execvp(args[0], args); 
			}
		}
		
		//

		printf("Virtual Exec Redirect done ... !\n"); 
	}
}

