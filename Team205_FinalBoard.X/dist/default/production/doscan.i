# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Users/DJMik/.mchp_packs/Microchip/PIC18F-K_DFP/1.8.249/xc8\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2


# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\ctype.h" 1 3







# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\features.h" 1 3
# 8 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\ctype.h" 2 3


int isalnum(int);
int isalpha(int);
int isblank(int);
int iscntrl(int);
int isdigit(int);
int isgraph(int);
int islower(int);
int isprint(int);
int ispunct(int);
int isspace(int);
int isupper(int);
int isxdigit(int);
int tolower(int);
int toupper(int);
# 3 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\errno.h" 1 3
# 12 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\errno.h" 3
extern int errno;
# 4 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdarg.h" 1 3




# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\musl_xc8.h" 1 3
# 5 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdarg.h" 2 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3





typedef void * va_list[1];
# 168 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __int24 int24_t;
# 204 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 8 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdarg.h" 2 3


#pragma intrinsic(__va_start)
#pragma intrinsic(__va_arg)

extern void * __va_start(void);
extern void * __va_arg(void *, ...);
# 5 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stddef.h" 1 3
# 19 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stddef.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 18 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int wchar_t;
# 122 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned size_t;
# 132 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef int ptrdiff_t;
# 19 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stddef.h" 2 3
# 6 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 1 3
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 173 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 209 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 2 3


typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdint.h" 2 3
# 7 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdio.h" 1 3
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdio.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 11 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef void * __isoc_va_list[1];
# 137 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long ssize_t;
# 246 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long off_t;
# 399 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 24 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdio.h" 2 3
# 52 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);
int ungetc(int, FILE *);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

__attribute__((__format__(__printf__, 1, 2)))
int printf(const char *restrict, ...);
__attribute__((__format__(__printf__, 2, 3)))
int fprintf(FILE *restrict, const char *restrict, ...);
__attribute__((__format__(__printf__, 2, 3)))
int sprintf(char *restrict, const char *restrict, ...);
__attribute__((__format__(__printf__, 3, 4)))
int snprintf(char *restrict, size_t, const char *restrict, ...);

__attribute__((__format__(__printf__, 1, 0)))
int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__printf__, 2, 0)))
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__printf__, 3, 0)))
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

__attribute__((__format__(__scanf__, 1, 2)))
int scanf(const char *restrict, ...);
__attribute__((__format__(__scanf__, 2, 3)))
int fscanf(FILE *restrict, const char *restrict, ...);
__attribute__((__format__(__scanf__, 2, 3)))
int sscanf(const char *restrict, const char *restrict, ...);

__attribute__((__format__(__scanf__, 1, 0)))
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__scanf__, 2, 0)))
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 8 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 1 3
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 21 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\stdlib.h" 2 3


int atoi (const char *);
long atol (const char *);
long long atoll (const char *);
double atof (const char *);

float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);
long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);

int rand (void);
void srand (unsigned);

void *malloc (size_t);
void *calloc (size_t, size_t);
void *realloc (void *, size_t);
void free (void *);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));







__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);
long long llabs (long long);

typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;
typedef struct { long long quot, rem; } lldiv_t;

div_t div (int, int);
ldiv_t ldiv (long, long);
lldiv_t lldiv (long long, long long);

typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);
# 9 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 1 3
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 411 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\bits/alltypes.h" 3
typedef struct __locale_struct * locale_t;
# 25 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 2 3


void *memcpy (void *restrict, const void *restrict, size_t);
void *memmove (void *, const void *, size_t);
void *memset (void *, int, size_t);
int memcmp (const void *, const void *, size_t);
void *memchr (const void *, int, size_t);

char *strcpy (char *restrict, const char *restrict);
char *strncpy (char *restrict, const char *restrict, size_t);

char *strcat (char *restrict, const char *restrict);
char *strncat (char *restrict, const char *restrict, size_t);

int strcmp (const char *, const char *);
int strncmp (const char *, const char *, size_t);

int strcoll (const char *, const char *);
size_t strxfrm (char *restrict, const char *restrict, size_t);

char *strchr (const char *, int);
char *strrchr (const char *, int);

size_t strcspn (const char *, const char *);
size_t strspn (const char *, const char *);
char *strpbrk (const char *, const char *);
char *strstr (const char *, const char *);
char *strtok (char *restrict, const char *restrict);

size_t strlen (const char *);

char *strerror (int);
# 65 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\c99\\string.h" 3
char *strtok_r (char *restrict, const char *restrict, char **restrict);
int strerror_r (int, char *, size_t);
char *stpcpy(char *restrict, const char *restrict);
char *stpncpy(char *restrict, const char *restrict, size_t);
size_t strnlen (const char *, size_t);
char *strdup (const char *);
char *strndup (const char *, size_t);
char *strsignal(int);
char *strerror_l (int, locale_t);
int strcoll_l (const char *, const char *, locale_t);
size_t strxfrm_l (char *restrict, const char *restrict, size_t, locale_t);




void *memccpy (void *restrict, const void *restrict, int, size_t);
# 10 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\include\\inline.h" 1 3
# 11 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c" 2
# 51 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
static int asup, width;


static int ncnv, nmatch;
# 64 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
static char dbuf[80];


static void skipws(FILE *fp)
{
    int c;

    while ((c = fgetc(fp)) != (-1)) {
        if (!(0 && isspace(c), ((c == ' ') || ((unsigned)(c)-'\t' < 5)))) {
            ungetc(c, fp);
            break;
        }
    }
}
# 263 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
static int atoaefg(FILE *fp, long double *pld)
{
    char *ep;
    int c, d, e, i, s, w;


    skipws(fp);


    w = width ? width : sizeof(dbuf) - 1;
    d = e = i = s = 0;
    while ((i < w) && (i < (sizeof(dbuf) - 1))) {
        c = fgetc(fp);
        if (c == (-1) && (width || i==0)) {
            return c;
        }
        if (!s && ((c == '+') || (c == '-'))) {
            s = 1;
            dbuf[i] = (char)c;
            ++i;
            ++nmatch;
            continue;
        }
        if (!d && (c == '.')) {
            d = 1;
            dbuf[i] = (char)c;
            ++i;
            ++nmatch;
            continue;
        }
        if (!e && ((c == 'e') || (c == 'E') || (c == 'p') || (c == 'P'))) {
            e = 1;
            s = 0;
            dbuf[i] = (char)c;
            ++i;
            ++nmatch;
            continue;
        }
        if (!(0 && isdigit(c), ((unsigned)(c)-'0') < 10)) {
   if (c != (-1)) {
    ungetc(c, fp);
   }
            break;
        }
        dbuf[i] = (char)c;
        ++i;
        ++nmatch;
    }
    dbuf[i] = '\0';



    *pld = strtof(&dbuf[0],&ep);

    return (errno || !i || ep == &dbuf[0]) ? (-1) - 1 : 1;
}
# 514 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
static int vfsfcnvrt(FILE *fp, char *fmt[], va_list ap)
{
    char *cp, ct[3];
    int c, i;
    long long ll;
    unsigned long long llu;
    long double f;
    void *vp;


    if ((*fmt)[0] == '%') {
        ++*fmt;

        asup = width = 0;
# 547 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
        ct[0] = (char)tolower((int)(*fmt)[0]);
        if (ct[0]) {
            ct[1] = (char)tolower((int)(*fmt)[1]);
            if (ct[1]) {
                ct[2] = (char)tolower((int)(*fmt)[2]);
            }
        }
# 797 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
        if ((ct[0] == 'a') || (ct[0] == 'e') || (ct[0] == 'f') || (ct[0] == 'g')) {


            ++*fmt;
            i = atoaefg(fp, &f);
            if (!asup && !(i < 1)) {
                vp = (void *)(*(float * *)__va_arg(*(float * **)ap, (float *)0));
                *(float *)vp = (float)f;
            }
            return i;
        }
        if (!strncmp(ct, "la", ((sizeof("le")/sizeof("le"[0]))-1)) || !strncmp(ct, "le", ((sizeof("lf")/sizeof("lf"[0]))-1)) || !strncmp(ct, "lf", ((sizeof("lf")/sizeof("lf"[0]))-1)) || !strncmp(ct, "lg", ((sizeof("lg")/sizeof("lg"[0]))-1))) {




            i = atoaefg(fp, &f);
            if (!asup && !(i < 1)) {
                if ((0 && isupper((int)(*fmt)[0]), ((unsigned)((int)(*fmt)[0])-'A') < 26)) {
                    vp = (void *)(*(long double * *)__va_arg(*(long double * **)ap, (long double *)0));
                    *(long double *)vp = f;
                } else {
                    vp = (void *)(*(double * *)__va_arg(*(double * **)ap, (double *)0));
                    *(double *)vp = (double)f;
                }
            }
            *fmt += ((sizeof("la")/sizeof("la"[0]))-1);
            return i;
        }
# 1287 "C:\\Program Files\\Microchip\\xc8\\v2.40\\pic\\sources\\c99\\common\\doscan.c"
        if ((*fmt)[0] == '%') {
   skipws(fp);
   c = fgetc(fp);
   if (c == '%') {
    ++*fmt;
    ++nmatch;
    return 0;
   }
   else if (c != (-1)) {
    ungetc(c, fp);
   }
   return (-1) - 1;
        }


        ++*fmt;
        return 0;
    }


    c = fgetc(fp);
 if ((0 && isspace(c), ((c == ' ') || ((unsigned)(c)-'\t' < 5))) && (0 && isspace(*fmt[0]), ((*fmt[0] == ' ') || ((unsigned)(*fmt[0])-'\t' < 5)))) {

  do {
   ++*fmt;
  } while((0 && isspace(*fmt[0]), ((*fmt[0] == ' ') || ((unsigned)(*fmt[0])-'\t' < 5))));
  do {
   ++nmatch;
   c = fgetc(fp);
  } while ((0 && isspace(c), ((c == ' ') || ((unsigned)(c)-'\t' < 5))));
  if (c != (-1)) {
   ungetc(c, fp);
  }
 }
 else if (c == *fmt[0]) {
        ++*fmt;
        ++nmatch;
    } else {
  if (c == (-1)) {
   return (-1);
  }
        ungetc(c, fp);
        return (-1) - 1;
    }

    return 0;
}

int vfscanf(FILE *fp, const char *fmt, va_list ap)
{
    char *cfmt;
    int n, saved_errno;

    cfmt = (char *)fmt;
    ncnv = nmatch = 0;
 saved_errno = errno;
    while (*cfmt) {
  errno = 0;
        n = vfsfcnvrt(fp, &cfmt, ap);
        if (n < 0) {
            break;
        }
        ncnv += n;
    }
 errno = saved_errno;
    if (n == (-1) && ncnv == 0) {
        return n;
    }

    return ncnv;
}
