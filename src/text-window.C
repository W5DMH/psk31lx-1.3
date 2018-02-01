#include <string.h>

#include <curses.h>
#include <ncurses.h>

#include "text-window.h"

void window::init_window() {
    initscr();
    cbreak();
    noecho();
    nonl();
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
}

void window::end_window() {
    endwin();
}

void window::putsyx(int line, int col, char *txt)
{
    int x=0,y=0;
    getsyx(x,y);
    move(line,col);
    addstr((char *)txt);
    move(x,y);
    refresh();
    doupdate();
}

void window::outputline(int line, char *txt) {
    int x=0,y=0;
    getsyx(x,y);
    move(line,0);
    clrtoeol();
    addstr((char *)txt);
    move(x,y);
    refresh();
    doupdate();
}


void window::initline(char **buffer) {
    *buffer=new char[width+1];
    bzero(*buffer,width+1);
}
void window::set_cursor() {
    set_cursor(cur_line, cur_col);
}

void window::set_cursor(int line, int column) {
    if(have_cursor) {
        move(ypos+line, column);
        refresh();
        doupdate();
    }
}

void window::paint_char(int line, int column) {
    int x=0,y=0;
    if(!have_cursor) getsyx(y,x);
    move(ypos+line, column);
    if(buffer[line]) addch((unsigned char)buffer[line][column]);
    if(!have_cursor) move(y,x);
    refresh();
    doupdate();
}
void window::paint_line(int line) {
    int x=0,y=0;
    if(!have_cursor) getsyx(y,x);
    move(ypos+line, 0);
    clrtoeol();
    if(buffer[line]) addstr((char *)buffer[line]);
    move(ypos+cur_line, cur_col);
    if(!have_cursor) move(y,x);
    refresh();
    doupdate();
}


void window::paint_all() {
    int x=0,y=0;
    if(!have_cursor) getsyx(y,x);
    move(ypos, 0);
    for(int i=0; i<height; i++) {
        clrtoeol();
        if(buffer[i]) addstr((char *)buffer[i]);
        addch('\n');
    }
    move(ypos+cur_line, cur_col);
    if(!have_cursor) move(y,x);
    refresh();
    doupdate();
}

window::window(int top, int h, int w, int c) {
    cur_line=cur_col=0;
    height=h;
    width=w;
    buffer=new char *[h];
    ypos=top;
    for(int i=0; i<h; i++) buffer[i]=(char *)0;
    have_cursor=c;
    if(have_cursor) {
        move(ypos+cur_line, cur_col);
        refresh();
        doupdate();
    }

}
void window::put_char(char c) {
    int lin=cur_line,col=cur_col;
    if(!buffer[cur_line]) initline(buffer+cur_line);
    if(c=='\n'||c=='\r')
        new_line();
    else if(c=='\b') {
        if(cur_col==0) return;
        cur_col--;
        buffer[cur_line][cur_col]=' ';
        paint_char(cur_line, cur_col);
        set_cursor(cur_line,cur_col);
    }
    else {
        buffer[cur_line][cur_col]=c;
        cur_col++;
        if(cur_col>=width-1) {
            if( new_line() ) return; /* ==1, falls redraw */
        }
        paint_char(lin,col); /* draw char */
    }
}

void window::put_string(char *s) {
    if(!buffer[cur_line]) initline(buffer+cur_line);
    while(*s) {
        put_char(*s);
        s++;
    };
}

void window::clear_line(int n) {
    if(n<0 || n>=height) return;
    if(buffer[n]) bzero(buffer[n],width+1);
    cur_line=n;
    cur_col=0;
    paint_line(n);   /* redraw line */
}

void window::put_line(char *l) {
    put_line(cur_line, l);
    new_line();
}

void window::put_line(int n, char *l) {
    if(n<0 || n>=height) return;
    if(!buffer[n]) initline(buffer+n);
    strncpy(buffer[n], (char *)l, width);
    paint_line(n);
}

void window::clear_all() {
    for(int i=0; i<height; i++) {
        if(buffer[i]) bzero(buffer[i],width+1);
    }
    cur_line=cur_col=0;
    paint_all();
}

int window::new_line() {
    cur_col=0;
    cur_line++;
    if(cur_line>=height) {
        int i;
        delete buffer[0];
        for(i=0; i<height-1; i++)
            buffer[i]=buffer[i+1];
        buffer[i]=(char *)NULL;
        cur_line--;
        paint_all();	/* redraw window */
        return 1;
    }
    set_cursor(cur_line, cur_col);
    return 0;
}
