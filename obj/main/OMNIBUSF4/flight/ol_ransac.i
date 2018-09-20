# 1 "./src/main/flight/ol_ransac.c"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "./src/main/flight/ol_ransac.c"
# 1 "./src/main/flight/ol_ransac.h" 1
struct dronerace_ransac_struct
{

  float dt_max;


  int buf_index_of_last;
  int buf_size;
};

extern struct dronerace_ransac_struct dr_ransac;



extern void ransac_reset(void);
extern void ransac_update_buffer_size(void);
extern void ransac_push(float time, float x, float y, float mx, float my);
# 2 "./src/main/flight/ol_ransac.c" 2
# 1 "./src/main/flight/ol_filter.h" 1
struct dronerace_vision_struct
{
  int cnt;
  float dx;
  float dy;
  float dz;
};


extern struct dronerace_vision_struct dr_vision;

struct dronerace_state_struct
{

  float time;


  float x;
  float y;


  float vx;
  float vy;


  float psi;
};

extern struct dronerace_state_struct dr_state;
extern float ol_dt;

extern void ol_filter_reset(void);

extern void ol_filter_predict(void);
extern void ol_filter_correct(void);
# 3 "./src/main/flight/ol_ransac.c" 2




struct dronerace_ransac_buf_struct
{

  float time;


  float x;
  float y;


  float mx;
  float my;
};

struct dronerace_ransac_buf_struct ransac_buf[20];

struct dronerace_ransac_struct dr_ransac;


void ransac_reset(void)
{
  int i;
  for (i=0; i<20; i++) {
    ransac_buf[i].time = 0;
    ransac_buf[i].x = 0;
    ransac_buf[i].y = 0;
    ransac_buf[i].mx = 0;
    ransac_buf[i].my = 0;
  }
  dr_ransac.dt_max = 1.0;
}


static int get_index(int element)
{
  int ind = dr_ransac.buf_index_of_last - element;
  if (ind < 0) { ind += 20; }
  return ind;
}

void ransac_update_buffer_size(void)
{
  int i;


  dr_ransac.buf_size = 0;
  for (i=0; i<20; i++ )
  {
    float mt = ransac_buf[get_index(i)].time;
    if ((mt == 0) || ((dr_state.time - mt) > dr_ransac.dt_max))
    {
      break;
    }
    dr_ransac.buf_size++;
  }
}

void ransac_push(float time, float x, float y, float mx, float my)
{
  int i = 0;
  static int ransac_cnt = 0;


  dr_ransac.buf_index_of_last++;
  if (dr_ransac.buf_index_of_last >= 20)
  {
    dr_ransac.buf_index_of_last = 0;
  }
  ransac_buf[dr_ransac.buf_index_of_last].time = time;
  ransac_buf[dr_ransac.buf_index_of_last].x = x;
  ransac_buf[dr_ransac.buf_index_of_last].y = y;
  ransac_buf[dr_ransac.buf_index_of_last].mx = mx;
  ransac_buf[dr_ransac.buf_index_of_last].my = my;





  if (dr_ransac.buf_size > 4)
  {
# 100 "./src/main/flight/ol_ransac.c"
    int n_samples = ((int)dr_ransac.buf_size * 0.4);
    int n_iterations = 200;
    float error_threshold = 1.0;
    int D = 1;
    int count = dr_ransac.buf_size;
    float targets_x[count];
    float targets_y[count];
    float samples[count][1];
    float params_x[2];
    float params_y[2];
    float fit_error;

    for (i=0;i<count;i++)
    {
      struct dronerace_ransac_buf_struct* r = &ransac_buf[get_index(i)];
      targets_x[i] = r->x - r->mx;
      targets_y[i] = r->y - r->my;
      samples[i][0] = r->time - dr_state.time;

    }
# 129 "./src/main/flight/ol_ransac.c"
    ransac_cnt ++;
# 158 "./src/main/flight/ol_ransac.c"
  }
}

# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 1 3
# 29 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_ansi.h" 1 3
# 15 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_ansi.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/newlib.h" 1 3
# 14 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/newlib.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_newlib_version.h" 1 3
# 15 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/newlib.h" 2 3
# 16 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_ansi.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/config.h" 1 3



# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/ieeefp.h" 1 3
# 5 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/config.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/features.h" 1 3
# 6 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/config.h" 2 3
# 17 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_ansi.h" 2 3
# 30 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3





# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/cdefs.h" 1 3
# 43 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/cdefs.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 1 3
# 41 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3

# 41 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef signed char __int8_t;

typedef unsigned char __uint8_t;
# 55 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef short int __int16_t;

typedef short unsigned int __uint16_t;
# 77 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef long int __int32_t;

typedef long unsigned int __uint32_t;
# 103 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef long long int __int64_t;

typedef long long unsigned int __uint64_t;
# 134 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef signed char __int_least8_t;

typedef unsigned char __uint_least8_t;
# 160 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef short int __int_least16_t;

typedef short unsigned int __uint_least16_t;
# 182 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef long int __int_least32_t;

typedef long unsigned int __uint_least32_t;
# 200 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef long long int __int_least64_t;

typedef long long unsigned int __uint_least64_t;
# 214 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_default_types.h" 3
typedef long long int __intmax_t;







typedef long long unsigned int __uintmax_t;







typedef int __intptr_t;

typedef unsigned int __uintptr_t;
# 44 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/cdefs.h" 2 3

# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 1 3 4
# 216 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 3 4
typedef unsigned int size_t;
# 46 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/cdefs.h" 2 3
# 36 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 1 3 4
# 149 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 3 4
typedef int ptrdiff_t;
# 328 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 3 4
typedef unsigned int wchar_t;
# 37 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3



# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stdarg.h" 1 3 4
# 40 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stdarg.h" 3 4
typedef __builtin_va_list __gnuc_va_list;
# 41 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3





typedef __gnuc_va_list va_list;
# 60 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 1 3
# 13 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/_ansi.h" 1 3
# 14 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 1 3 4
# 15 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 1 3
# 24 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_types.h" 1 3
# 25 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/lock.h" 1 3
# 33 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/lock.h" 3
struct __lock;
typedef struct __lock * _LOCK_T;






extern void __retarget_lock_init(_LOCK_T *lock);

extern void __retarget_lock_init_recursive(_LOCK_T *lock);

extern void __retarget_lock_close(_LOCK_T lock);

extern void __retarget_lock_close_recursive(_LOCK_T lock);

extern void __retarget_lock_acquire(_LOCK_T lock);

extern void __retarget_lock_acquire_recursive(_LOCK_T lock);

extern int __retarget_lock_try_acquire(_LOCK_T lock);

extern int __retarget_lock_try_acquire_recursive(_LOCK_T lock);


extern void __retarget_lock_release(_LOCK_T lock);

extern void __retarget_lock_release_recursive(_LOCK_T lock);
# 26 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 2 3


typedef long __blkcnt_t;



typedef long __blksize_t;



typedef __uint64_t __fsblkcnt_t;



typedef __uint32_t __fsfilcnt_t;



typedef long _off_t;





typedef int __pid_t;



typedef short __dev_t;



typedef unsigned short __uid_t;


typedef unsigned short __gid_t;



typedef __uint32_t __id_t;







typedef unsigned short __ino_t;
# 88 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 3
typedef __uint32_t __mode_t;





__extension__ typedef long long _off64_t;





typedef _off_t __off_t;


typedef _off64_t __loff_t;


typedef long __key_t;







typedef long _fpos_t;
# 129 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 3
typedef unsigned int __size_t;
# 145 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 3
typedef signed int _ssize_t;
# 156 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 3
typedef _ssize_t __ssize_t;


# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 1 3 4
# 357 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 3 4
typedef unsigned int wint_t;
# 160 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_types.h" 2 3



typedef struct
{
  int __count;
  union
  {
    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;



typedef _LOCK_T _flock_t;




typedef void *_iconv_t;






typedef unsigned long __clock_t;






typedef __int_least64_t __time_t;


typedef unsigned long __clockid_t;


typedef unsigned long __timer_t;


typedef __uint8_t __sa_family_t;



typedef __uint32_t __socklen_t;


typedef unsigned short __nlink_t;
typedef long __suseconds_t;
typedef unsigned long __useconds_t;


typedef __builtin_va_list __va_list;
# 16 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 2 3






typedef unsigned long __ULong;
# 38 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
struct _reent;

struct __locale_t;






struct _Bigint
{
  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm
{
  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _on_exit_args {
 void * _fnargs[32];
 void * _dso_handle[32];

 __ULong _fntypes;


 __ULong _is_cxa;
};
# 93 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
struct _atexit {
 struct _atexit *_next;
 int _ind;

 void (*_fns[32])(void);
        struct _on_exit_args _on_exit_args;
};
# 117 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
struct __sbuf {
 unsigned char *_base;
 int _size;
};
# 181 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  void * _cookie;

  int (* _read) (struct _reent *, void *, char *, int)
                                          ;
  int (* _write) (struct _reent *, void *, const char *, int)

                                   ;
  _fpos_t (* _seek) (struct _reent *, void *, _fpos_t, int);
  int (* _close) (struct _reent *, void *);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  _off_t _offset;


  struct _reent *_data;



  _flock_t _lock;

  _mbstate_t _mbstate;
  int _flags2;
};
# 287 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
typedef struct __sFILE __FILE;



struct _glue
{
  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
# 319 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;




};
# 569 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
struct _reent
{
  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];


  int _unspecified_locale_info;
  struct __locale_t *_locale;

  int __sdidinit;

  void (* __cleanup) (struct _reent *);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union
    {
      struct
        {
          unsigned int _unused_rand;
          char * _strtok_last;
          char _asctime_buf[26];
          struct __tm _localtime_buf;
          int _gamma_signgam;
          __extension__ unsigned long long _rand_next;
          struct _rand48 _r48;
          _mbstate_t _mblen_state;
          _mbstate_t _mbtowc_state;
          _mbstate_t _wctomb_state;
          char _l64a_buf[8];
          char _signal_buf[24];
          int _getdate_err;
          _mbstate_t _mbrlen_state;
          _mbstate_t _mbrtowc_state;
          _mbstate_t _mbsrtowcs_state;
          _mbstate_t _wcrtomb_state;
          _mbstate_t _wcsrtombs_state;
   int _h_errno;
        } _reent;



      struct
        {

          unsigned char * _nextf[30];
          unsigned int _nmalloc[30];
        } _unused;
    } _new;



  struct _atexit *_atexit;
  struct _atexit _atexit0;



  void (**(_sig_func))(int);




  struct _glue __sglue;

  __FILE __sf[3];

};
# 775 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/reent.h" 3
extern struct _reent *_impure_ptr ;
extern struct _reent *const _global_impure_ptr ;

void _reclaim_reent (struct _reent *);
# 61 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 1 3
# 28 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 3
typedef __uint8_t u_int8_t;


typedef __uint16_t u_int16_t;


typedef __uint32_t u_int32_t;


typedef __uint64_t u_int64_t;

typedef int register_t;
# 62 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/lib/gcc/arm-none-eabi/7.2.1/include/stddef.h" 1 3 4
# 63 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 2 3

# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_stdint.h" 1 3
# 20 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_stdint.h" 3
typedef __int8_t int8_t ;



typedef __uint8_t uint8_t ;







typedef __int16_t int16_t ;



typedef __uint16_t uint16_t ;







typedef __int32_t int32_t ;



typedef __uint32_t uint32_t ;







typedef __int64_t int64_t ;



typedef __uint64_t uint64_t ;






typedef __intmax_t intmax_t;




typedef __uintmax_t uintmax_t;




typedef __intptr_t intptr_t;




typedef __uintptr_t uintptr_t;
# 65 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 2 3


# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/endian.h" 1 3





# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/_endian.h" 1 3
# 7 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/endian.h" 2 3
# 68 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 1 3
# 25 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_sigset.h" 1 3
# 41 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_sigset.h" 3
typedef unsigned long __sigset_t;
# 26 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_timeval.h" 1 3
# 35 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_timeval.h" 3
typedef __suseconds_t suseconds_t;




typedef __int_least64_t time_t;
# 52 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_timeval.h" 3
struct timeval {
 time_t tv_sec;
 suseconds_t tv_usec;
};
# 27 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/timespec.h" 1 3
# 38 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/timespec.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_timespec.h" 1 3
# 45 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_timespec.h" 3
struct timespec {
 time_t tv_sec;
 long tv_nsec;
};
# 39 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/timespec.h" 2 3
# 58 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/timespec.h" 3
struct itimerspec {
 struct timespec it_interval;
 struct timespec it_value;
};
# 28 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 2 3



typedef __sigset_t sigset_t;
# 45 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 3
typedef unsigned long fd_mask;







typedef struct _types_fd_set {
 fd_mask fds_bits[(((64)+(((sizeof (fd_mask) * 8))-1))/((sizeof (fd_mask) * 8)))];
} _types_fd_set;
# 71 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/select.h" 3


int select (int __n, _types_fd_set *__readfds, _types_fd_set *__writefds, _types_fd_set *__exceptfds, struct timeval *__timeout)
                                                   ;

int pselect (int __n, _types_fd_set *__readfds, _types_fd_set *__writefds, _types_fd_set *__exceptfds, const struct timespec *__timeout, const sigset_t *__set)

                           ;



# 69 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 2 3




typedef __uint32_t in_addr_t;




typedef __uint16_t in_port_t;
# 87 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 3
typedef unsigned char u_char;



typedef unsigned short u_short;



typedef unsigned int u_int;



typedef unsigned long u_long;







typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;



typedef __blkcnt_t blkcnt_t;




typedef __blksize_t blksize_t;




typedef unsigned long clock_t;
# 135 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 3
typedef long daddr_t;



typedef char * caddr_t;




typedef __fsblkcnt_t fsblkcnt_t;
typedef __fsfilcnt_t fsfilcnt_t;




typedef __id_t id_t;




typedef __ino_t ino_t;
# 173 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 3
typedef __off_t off_t;



typedef __dev_t dev_t;



typedef __uid_t uid_t;



typedef __gid_t gid_t;




typedef __pid_t pid_t;




typedef __key_t key_t;




typedef _ssize_t ssize_t;




typedef __mode_t mode_t;




typedef __nlink_t nlink_t;




typedef __clockid_t clockid_t;





typedef __timer_t timer_t;





typedef __useconds_t useconds_t;
# 236 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 3
typedef __int64_t sbintime_t;


# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 1 3
# 23 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/sched.h" 1 3
# 48 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/sched.h" 3
struct sched_param {
  int sched_priority;
# 61 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/sched.h" 3
};
# 24 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 2 3
# 32 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 3
typedef __uint32_t pthread_t;
# 61 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 3
typedef struct {
  int is_initialized;
  void *stackaddr;
  int stacksize;
  int contentionscope;
  int inheritsched;
  int schedpolicy;
  struct sched_param schedparam;





  int detachstate;
} pthread_attr_t;
# 154 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 3
typedef __uint32_t pthread_mutex_t;

typedef struct {
  int is_initialized;
# 168 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/_pthreadtypes.h" 3
  int recursive;
} pthread_mutexattr_t;






typedef __uint32_t pthread_cond_t;



typedef struct {
  int is_initialized;
  clock_t clock;



} pthread_condattr_t;



typedef __uint32_t pthread_key_t;

typedef struct {
  int is_initialized;
  int init_executed;
} pthread_once_t;
# 240 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 2 3
# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/machine/types.h" 1 3
# 241 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/types.h" 2 3
# 62 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3




typedef __FILE FILE;






typedef _fpos_t fpos_t;





# 1 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/sys/stdio.h" 1 3
# 80 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 2 3
# 181 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
char * ctermid (char *);




FILE * tmpfile (void);
char * tmpnam (char *);

char * tempnam (const char *, const char *);

int fclose (FILE *);
int fflush (FILE *);
FILE * freopen (const char *restrict, const char *restrict, FILE *restrict);
void setbuf (FILE *restrict, char *restrict);
int setvbuf (FILE *restrict, char *restrict, int, size_t);
int fprintf (FILE *restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
int fscanf (FILE *restrict, const char *restrict, ...) __attribute__ ((__format__ (__scanf__, 2, 3)))
                                                           ;
int printf (const char *restrict, ...) __attribute__ ((__format__ (__printf__, 1, 2)))
                                                            ;
int scanf (const char *restrict, ...) __attribute__ ((__format__ (__scanf__, 1, 2)))
                                                           ;
int sscanf (const char *restrict, const char *restrict, ...) __attribute__ ((__format__ (__scanf__, 2, 3)))
                                                           ;
int vfprintf (FILE *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int vprintf (const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 1, 0)))
                                                            ;
int vsprintf (char *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int fgetc (FILE *);
char * fgets (char *restrict, int, FILE *restrict);
int fputc (int, FILE *);
int fputs (const char *restrict, FILE *restrict);
int getc (FILE *);
int getchar (void);
char * gets (char *);
int putc (int, FILE *);
int putchar (int);
int puts (const char *);
int ungetc (int, FILE *);
size_t fread (void * restrict, size_t _size, size_t _n, FILE *restrict);
size_t fwrite (const void * restrict , size_t _size, size_t _n, FILE *);



int fgetpos (FILE *restrict, fpos_t *restrict);

int fseek (FILE *, long, int);



int fsetpos (FILE *, const fpos_t *);

long ftell ( FILE *);
void rewind (FILE *);
void clearerr (FILE *);
int feof (FILE *);
int ferror (FILE *);
void perror (const char *);

FILE * fopen (const char *restrict _name, const char *restrict _type);
int sprintf (char *restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
int remove (const char *);
int rename (const char *, const char *);
# 257 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
int fseeko (FILE *, off_t, int);
off_t ftello ( FILE *);







int snprintf (char *restrict, size_t, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int vsnprintf (char *restrict, size_t, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int vfscanf (FILE *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 2, 0)))
                                                           ;
int vscanf (const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 1, 0)))
                                                           ;
int vsscanf (const char *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 2, 0)))
                                                           ;
# 284 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
int asiprintf (char **, const char *, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
char * asniprintf (char *, size_t *, const char *, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
char * asnprintf (char *restrict, size_t *restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;

int diprintf (int, const char *, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;

int fiprintf (FILE *, const char *, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
int fiscanf (FILE *, const char *, ...) __attribute__ ((__format__ (__scanf__, 2, 3)))
                                                           ;
int iprintf (const char *, ...) __attribute__ ((__format__ (__printf__, 1, 2)))
                                                            ;
int iscanf (const char *, ...) __attribute__ ((__format__ (__scanf__, 1, 2)))
                                                           ;
int siprintf (char *, const char *, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
int siscanf (const char *, const char *, ...) __attribute__ ((__format__ (__scanf__, 2, 3)))
                                                           ;
int sniprintf (char *, size_t, const char *, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int vasiprintf (char **, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
char * vasniprintf (char *, size_t *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
char * vasnprintf (char *, size_t *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int vdiprintf (int, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int vfiprintf (FILE *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int vfiscanf (FILE *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 2, 0)))
                                                           ;
int viprintf (const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 1, 0)))
                                                            ;
int viscanf (const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 1, 0)))
                                                           ;
int vsiprintf (char *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int vsiscanf (const char *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 2, 0)))
                                                           ;
int vsniprintf (char *, size_t, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
# 339 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
FILE * fdopen (int, const char *);

int fileno (FILE *);


int pclose (FILE *);
FILE * popen (const char *, const char *);



void setbuffer (FILE *, char *, int);
int setlinebuf (FILE *);



int getw (FILE *);
int putw (int, FILE *);


int getc_unlocked (FILE *);
int getchar_unlocked (void);
void flockfile (FILE *);
int ftrylockfile (FILE *);
void funlockfile (FILE *);
int putc_unlocked (int, FILE *);
int putchar_unlocked (int);
# 374 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
int dprintf (int, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;

FILE * fmemopen (void *restrict, size_t, const char *restrict);


FILE * open_memstream (char **, size_t *);
int vdprintf (int, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;



int renameat (int, const char *, int, const char *);
# 396 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
int _asiprintf_r (struct _reent *, char **, const char *, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
char * _asniprintf_r (struct _reent *, char *, size_t *, const char *, ...) __attribute__ ((__format__ (__printf__, 4, 5)))
                                                            ;
char * _asnprintf_r (struct _reent *, char *restrict, size_t *restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 4, 5)))
                                                            ;
int _asprintf_r (struct _reent *, char **restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _diprintf_r (struct _reent *, int, const char *, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _dprintf_r (struct _reent *, int, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _fclose_r (struct _reent *, FILE *);
int _fcloseall_r (struct _reent *);
FILE * _fdopen_r (struct _reent *, int, const char *);
int _fflush_r (struct _reent *, FILE *);
int _fgetc_r (struct _reent *, FILE *);
int _fgetc_unlocked_r (struct _reent *, FILE *);
char * _fgets_r (struct _reent *, char *restrict, int, FILE *restrict);
char * _fgets_unlocked_r (struct _reent *, char *restrict, int, FILE *restrict);




int _fgetpos_r (struct _reent *, FILE *, fpos_t *);
int _fsetpos_r (struct _reent *, FILE *, const fpos_t *);

int _fiprintf_r (struct _reent *, FILE *, const char *, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _fiscanf_r (struct _reent *, FILE *, const char *, ...) __attribute__ ((__format__ (__scanf__, 3, 4)))
                                                           ;
FILE * _fmemopen_r (struct _reent *, void *restrict, size_t, const char *restrict);
FILE * _fopen_r (struct _reent *, const char *restrict, const char *restrict);
FILE * _freopen_r (struct _reent *, const char *restrict, const char *restrict, FILE *restrict);
int _fprintf_r (struct _reent *, FILE *restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _fpurge_r (struct _reent *, FILE *);
int _fputc_r (struct _reent *, int, FILE *);
int _fputc_unlocked_r (struct _reent *, int, FILE *);
int _fputs_r (struct _reent *, const char *restrict, FILE *restrict);
int _fputs_unlocked_r (struct _reent *, const char *restrict, FILE *restrict);
size_t _fread_r (struct _reent *, void * restrict, size_t _size, size_t _n, FILE *restrict);
size_t _fread_unlocked_r (struct _reent *, void * restrict, size_t _size, size_t _n, FILE *restrict);
int _fscanf_r (struct _reent *, FILE *restrict, const char *restrict, ...) __attribute__ ((__format__ (__scanf__, 3, 4)))
                                                           ;
int _fseek_r (struct _reent *, FILE *, long, int);
int _fseeko_r (struct _reent *, FILE *, _off_t, int);
long _ftell_r (struct _reent *, FILE *);
_off_t _ftello_r (struct _reent *, FILE *);
void _rewind_r (struct _reent *, FILE *);
size_t _fwrite_r (struct _reent *, const void * restrict, size_t _size, size_t _n, FILE *restrict);
size_t _fwrite_unlocked_r (struct _reent *, const void * restrict, size_t _size, size_t _n, FILE *restrict);
int _getc_r (struct _reent *, FILE *);
int _getc_unlocked_r (struct _reent *, FILE *);
int _getchar_r (struct _reent *);
int _getchar_unlocked_r (struct _reent *);
char * _gets_r (struct _reent *, char *);
int _iprintf_r (struct _reent *, const char *, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
int _iscanf_r (struct _reent *, const char *, ...) __attribute__ ((__format__ (__scanf__, 2, 3)))
                                                           ;
FILE * _open_memstream_r (struct _reent *, char **, size_t *);
void _perror_r (struct _reent *, const char *);
int _printf_r (struct _reent *, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 2, 3)))
                                                            ;
int _putc_r (struct _reent *, int, FILE *);
int _putc_unlocked_r (struct _reent *, int, FILE *);
int _putchar_unlocked_r (struct _reent *, int);
int _putchar_r (struct _reent *, int);
int _puts_r (struct _reent *, const char *);
int _remove_r (struct _reent *, const char *);
int _rename_r (struct _reent *, const char *_old, const char *_new)
                                          ;
int _scanf_r (struct _reent *, const char *restrict, ...) __attribute__ ((__format__ (__scanf__, 2, 3)))
                                                           ;
int _siprintf_r (struct _reent *, char *, const char *, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _siscanf_r (struct _reent *, const char *, const char *, ...) __attribute__ ((__format__ (__scanf__, 3, 4)))
                                                           ;
int _sniprintf_r (struct _reent *, char *, size_t, const char *, ...) __attribute__ ((__format__ (__printf__, 4, 5)))
                                                            ;
int _snprintf_r (struct _reent *, char *restrict, size_t, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 4, 5)))
                                                            ;
int _sprintf_r (struct _reent *, char *restrict, const char *restrict, ...) __attribute__ ((__format__ (__printf__, 3, 4)))
                                                            ;
int _sscanf_r (struct _reent *, const char *restrict, const char *restrict, ...) __attribute__ ((__format__ (__scanf__, 3, 4)))
                                                           ;
char * _tempnam_r (struct _reent *, const char *, const char *);
FILE * _tmpfile_r (struct _reent *);
char * _tmpnam_r (struct _reent *, char *);
int _ungetc_r (struct _reent *, int, FILE *);
int _vasiprintf_r (struct _reent *, char **, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
char * _vasniprintf_r (struct _reent*, char *, size_t *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 4, 0)))
                                                            ;
char * _vasnprintf_r (struct _reent*, char *, size_t *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 4, 0)))
                                                            ;
int _vasprintf_r (struct _reent *, char **, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vdiprintf_r (struct _reent *, int, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vdprintf_r (struct _reent *, int, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vfiprintf_r (struct _reent *, FILE *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vfiscanf_r (struct _reent *, FILE *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 3, 0)))
                                                           ;
int _vfprintf_r (struct _reent *, FILE *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vfscanf_r (struct _reent *, FILE *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 3, 0)))
                                                           ;
int _viprintf_r (struct _reent *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int _viscanf_r (struct _reent *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 2, 0)))
                                                           ;
int _vprintf_r (struct _reent *, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 2, 0)))
                                                            ;
int _vscanf_r (struct _reent *, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 2, 0)))
                                                           ;
int _vsiprintf_r (struct _reent *, char *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vsiscanf_r (struct _reent *, const char *, const char *, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 3, 0)))
                                                           ;
int _vsniprintf_r (struct _reent *, char *, size_t, const char *, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 4, 0)))
                                                            ;
int _vsnprintf_r (struct _reent *, char *restrict, size_t, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 4, 0)))
                                                            ;
int _vsprintf_r (struct _reent *, char *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__printf__, 3, 0)))
                                                            ;
int _vsscanf_r (struct _reent *, const char *restrict, const char *restrict, __gnuc_va_list) __attribute__ ((__format__ (__scanf__, 3, 0)))
                                                           ;



int fpurge (FILE *);
ssize_t __getdelim (char **, size_t *, int, FILE *);
ssize_t __getline (char **, size_t *, FILE *);


void clearerr_unlocked (FILE *);
int feof_unlocked (FILE *);
int ferror_unlocked (FILE *);
int fileno_unlocked (FILE *);
int fflush_unlocked (FILE *);
int fgetc_unlocked (FILE *);
int fputc_unlocked (int, FILE *);
size_t fread_unlocked (void * restrict, size_t _size, size_t _n, FILE *restrict);
size_t fwrite_unlocked (const void * restrict , size_t _size, size_t _n, FILE *);
# 577 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
int __srget_r (struct _reent *, FILE *);
int __swbuf_r (struct _reent *, int, FILE *);
# 601 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
FILE *funopen (const void * __cookie, int (*__readfn)(void * __cookie, char *__buf, int __n), int (*__writefn)(void * __cookie, const char *__buf, int __n), fpos_t (*__seekfn)(void * __cookie, fpos_t __off, int __whence), int (*__closefn)(void * __cookie))





                                   ;
FILE *_funopen_r (struct _reent *, const void * __cookie, int (*__readfn)(void * __cookie, char *__buf, int __n), int (*__writefn)(void * __cookie, const char *__buf, int __n), fpos_t (*__seekfn)(void * __cookie, fpos_t __off, int __whence), int (*__closefn)(void * __cookie))





                                   ;
# 687 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
static __inline__ int __sputc_r(struct _reent *_ptr, int _c, FILE *_p) {




 if (--_p->_w >= 0 || (_p->_w >= _p->_lbfsize && (char)_c != '\n'))
  return (*_p->_p++ = _c);
 else
  return (__swbuf_r(_ptr, _c, _p));
}
# 741 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3
static __inline int
_getchar_unlocked(void)
{
 struct _reent *_ptr;

 _ptr = _impure_ptr;
 return ((--(((_ptr)->_stdin))->_r < 0 ? __srget_r(_ptr, ((_ptr)->_stdin)) : (int)(*(((_ptr)->_stdin))->_p++)));
}

static __inline int
_putchar_unlocked(int _c)
{
 struct _reent *_ptr;

 _ptr = _impure_ptr;
 return (__sputc_r(_ptr, _c, ((_ptr)->_stdout)));
}
# 797 "/home/jiahao/betaflight_autonomous/tools/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/include/stdio.h" 3

# 162 "./src/main/flight/ol_ransac.c" 2


# 163 "./src/main/flight/ol_ransac.c"
static void ransac_get_vector(void)
{
  int i = 0;
  for ( i=0; i<dr_ransac.buf_size; i++ )
  {

    ransac_buf[get_index(i)];
  }
}
