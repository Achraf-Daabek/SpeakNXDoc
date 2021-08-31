// C library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h> 
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

//define liste of known BT device connue 
#define NUMBER_OF_STRING 10
#define MAX_STRING_SIZE 20
#define MAX_BUFFER 1028



// Allocate memory for read buffer, set size according to your needs for data response
char read_buf [MAX_BUFFER];
//check if exist
char *ret;
//bytes to read
int num_bytes ;
//list of all known device 
char Action2n[NUMBER_OF_STRING][MAX_STRING_SIZE] =
  { "KeyPressed_ringing",
    "CodeValid",
    "CodeInValid",
    "NoiseDetected_Start",
    "NoiseDetected_End",
    "MotionDetected_Start",
    "MotionDetected_End",
    "relay1",
    "input1",
    "CAllState_Ringing"

  };


//fuction to config Serial port
void configSerialPort(int serial_port , unsigned int  baudrate){
 

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, baudrate);
  cfsetospeed(&tty, baudrate);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      
  }
  
}

  
int main() {
   // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyUSB1", O_RDWR);
  
  //call function to cinfig serial port
  configSerialPort(serial_port,B115200);

  //At command to config the web server 
  unsigned char command[]="AT+SRWSFWDHTTP=1\r";
   write(serial_port,command ,sizeof(command) -1);

  while(1){
     memset(&read_buf, '\0', sizeof(read_buf));
    
     num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      
  }
    //print Received message
     printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);
    
    for (int i = 0; i < NUMBER_OF_STRING; i++)
       
       {
         
        //check if command received
         ret = strstr(read_buf, Action2n[i]);
         
 
         if(ret != NULL) 
          {
            printf("ret is  : %s \n", ret);

           printf("%s Received\n",Action2n[i]);

          }
      }
      //sleep to 
  sleep(3);
  }
  


  
  //close serial port
  close(serial_port);
  return 0; // success
}

