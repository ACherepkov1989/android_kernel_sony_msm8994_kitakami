#ifndef _PARAM_ID_H_
#define _PARAM_ID_H_

#define PARAM_ID_LEN 0
#define PARAM_ID_OOM_ADJ 1
#define PARAM_ID_APP_PID 2
#define PARAM_ID_VALUE 3
#define PARAM_ID_RATE 4
#define PARAM_ID_SENSOR 5
#define PARAM_ID_SIZE 6
#define PARAM_ID_FD 7
#define NUM_PARAM_IDS 8

#ifndef PROVIDE_PARAM_ID
int param_id_index(const char *param, const char *end);
const char *get_param_id_name(int id);
#else
int param_id_index(const char *param, const char *end)
{
	int id  = -1;
	int len = ((end != NULL) ? (end - param) : (int)strlen(param));

	/**/ if ((len == 3) && !memcmp(param, "len", 3))
		id = 0;
	else if ((len == 7) && !memcmp(param, "oom_adj", 7))
		id = 1;
	else if ((len == 7) && !memcmp(param, "app_pid", 7))
		id = 2;
	else if ((len == 5) && !memcmp(param, "value", 5))
		id = 3;
	else if ((len == 4) && !memcmp(param, "rate", 4))
		id = 4;
	else if ((len == 6) && !memcmp(param, "sensor", 6))
		id = 5;
	else if ((len == 4) && !memcmp(param, "size", 4))
		id = 6;
	else if ((len == 2) && !memcmp(param, "fd", 2))
		id = 7;

	return id;
}

const char *get_param_id_name(int id)
{
	const char *name = "?";

	switch (id) {
	case 0:
		name = "len"; break;
	case 1:
		name = "oom_adj"; break;
	case 2:
		name = "app_pid"; break;
	case 3:
		name = "value"; break;
	case 4:
		name = "rate"; break;
	case 5:
		name = "sensor"; break;
	case 6:
		name = "size"; break;
	case 7:
		name = "fd"; break;
	}

	return name;
}
#endif /* PROVIDE_PARAM_ID */

#endif /* _PARAM_ID_H_ */
