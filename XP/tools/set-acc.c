#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <err.h>

/* Configure the attenuation. */

#define MAX_CMD_SIZE 255

int open_port(const char *dev)
{
  int fd; /* File descriptor for the port */
  struct termios tty;

  fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
    err(EXIT_FAILURE, "cannot open %s device", dev);
  else
    printf("port opened successfully\n");

  tcgetattr(fd, &tty);
  cfsetospeed (&tty, (speed_t)B9600);
  cfsetispeed (&tty, (speed_t)B9600);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CRTSCTS;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(INPCK | ISTRIP);
  tty.c_iflag &= ~(INLCR | ICRNL);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* shut off xon/xoff ctrl */
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN]=255;
  tty.c_cc[VTIME]=4;
  tcsetattr(fd, TCSAFLUSH, &tty);
  fcntl(fd,F_SETFL,0);
  printf("port configuration done\n");

  return fd;
}

static void write_cmd(int fd, const char *cmd)
{
  int n = write(fd, cmd, strlen(cmd));
  if(n <= 0)
    err(EXIT_FAILURE, "write() failed");
}

int main(int argc, const char *argv[])
{
  int fd;
  char cmd[MAX_CMD_SIZE];
  const char *ln, *att, *dev;

  if(argc != 3) {
    printf("usage: set_acc DEVICE LN ATTENUATION\n");
    exit(1);
  }

  dev = argv[1];
  ln  = argv[2];
  att = argv[3];

  /* set attenuation */
  fd=open_port(dev);

  write_cmd(fd, "onl\n");
  write_cmd(fd, "cac 0\n");

  snprintf(cmd, MAX_CMD_SIZE, "ln %s\n", ln);
  write_cmd(fd, cmd);

  snprintf(cmd,MAX_CMD_SIZE, "wrt %s \rA %s\n", ln, att);
  write_cmd(fd, cmd);

  return EXIT_SUCCESS;
}
