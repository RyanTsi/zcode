CC = gcc

SRCS = main.c list.c
OBJS = ${SRCS:.c=.o}
TARGET = fat12

all: ${TARGET}

${TARGET}: ${OBJS}
	${CC} -o ${TARGET} ${OBJS} -g

%.o: %.c
	${CC} -c $< -o $@

clean:
	rm -f ${TARGET} ${OBJS}