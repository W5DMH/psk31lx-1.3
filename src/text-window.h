
#ifndef __PSK31_WINDOW_INCLUDED
#define __PSK31_WINDOW_INCLUDED

class window {
private:
    int height, width, xpos, ypos;
    char **buffer;
    int cur_line, cur_col;
    int have_cursor;
    void initline(char **buffer);
    void paint_char(int line, int column);
    void paint_line(int line);
    void paint_all();
    void set_cursor(int line, int column);
    void set_cursor();
public:
    window(int topline, int h, int w, int cursor);
    void put_char(char c);
    void clear_line(int n);
    void put_line(char *l);
    void put_string(char *s);
    void put_line(int n, char *l);
    void clear_all();
    int new_line();
    static void outputline(int n, char *txt);
    static void putsyx(int y, int x, char *txt);
    static void init_window();
    static void end_window();
};

#endif
