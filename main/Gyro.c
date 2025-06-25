// Gyro app

#include <revk.h>

const char *
app_callback (int client, const char *prefix, const char *target, const char *suffix, jo_t j)
{
	return NULL;
}

void
revk_state_extra (jo_t j)
{
}

void
revk_web_extra (httpd_req_t * req, int page)
{
}

void
app_main ()
{
	revk_boot (&app_callback);
	revk_start();
}
