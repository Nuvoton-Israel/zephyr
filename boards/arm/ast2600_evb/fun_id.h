#ifndef __FUN_ID_H__
#define __FUN_ID_H__

/* concatenate the values of the arguments into one */
#define DO_CONCAT(x, y) x ## y
#define CONCAT(x, y) DO_CONCAT(x, y)

/* Pin ID */
enum {
	FUN_NONE = -1,
#define FUN_DEFINE(fun, ...) FUN_ ## fun,
	#include "fun_def_list.h"
#undef FUN_DEFINE
	MAX_FUN_ID,
};

#define GET_FUN_ID(fun) CONCAT(FUN_, fun)

#endif /* #ifndef __FUN_ID_H__ */
