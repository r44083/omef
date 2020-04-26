
#include <stdlib.h>
#include <string.h>
#include "syslog.hpp"
#include "common/assert.h"
#include "third_party/printf/printf.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace ul;

#define EOL "\r\n"
#define EOL_SIZE (sizeof(EOL) - 1)

syslog::syslog()
{
	ASSERT(api_lock = xSemaphoreCreateMutex());
}

void syslog::add_output(cb_t cb, void *ctx)
{
	ASSERT(cb);
	
	cb_ctx_t cb_ctx = {.cb = cb, .ctx = ctx};
	
	list_elem<cb_ctx_t> *elem = new list_elem<cb_ctx_t>(cb_ctx);
	ASSERT(elem);
	
	cb_ctx_list.add(elem);
}

void syslog::del_output(cb_t cb, void *ctx)
{
	if(cb_ctx_list.is_empty())
		return;
	
	cb_ctx_t cb_ctx = {.cb = cb, .ctx = ctx};
	
	list_elem<cb_ctx_t> *elem = cb_ctx_list.find(cb_ctx);
	if(elem)
	{
		cb_ctx_list.remove(elem);
		delete elem;
	}
}

void syslog::dbg(const char *format, ...)
{
	va_list args;
	
	va_start(args, format);
	print(DBG, format, args);
	va_end(args);
}

void syslog::inf(const char *format, ...)
{
	va_list args;
	
	va_start(args, format);
	print(INF, format, args);
	va_end(args);
}

void syslog::wrn(const char *format, ...)
{
	va_list args;
	
	va_start(args, format);
	print(INF, format, args);
	va_end(args);
}

void syslog::err(const char *format, ...)
{
	va_list args;
	
	va_start(args, format);
	print(INF, format, args);
	va_end(args);
}

void syslog::print(tag_t tag, const char *format, va_list va)
{
	if(cb_ctx_list.is_empty())
		return;
	
	// Protect message[] buffer with semaphore
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	size_t len = add_timestamp(message);
	len += add_tag(message + len, tag);
	
	len += vsnprintf_(message + len, sizeof(message) - len - EOL_SIZE, format, va);
	
	len += add_eol(message);
	
	for(list_elem<cb_ctx_t> *cb_ctx = cb_ctx_list.head(); cb_ctx;
		cb_ctx = cb_ctx->next())
	{
		cb_ctx->data().cb((uint8_t *)message, len, cb_ctx->data().ctx);
	}
	
	xSemaphoreGive(api_lock);
}

size_t syslog::add_tag(char *str, uint8_t tag)
{
	size_t len = strlen(tags[tag]);
	
	memcpy(str, tags[tag], len);
	
	return len;
}

size_t syslog::add_eol(char *str)
{
	strcat(str, EOL);
	
	return EOL_SIZE;
}

size_t syslog::add_timestamp(char *str)
{
	uint32_t ms = xTaskGetTickCount();// configTICK_RATE_HZ;
	uint16_t sss =  ms % 1000;
	uint8_t ss =   (ms / 1000) % 60;
	uint8_t mm =   (ms / 1000 / 60) % 60;
	uint8_t hh =   (ms / 1000 / 60 / 60) % 24;
	uint8_t DD =   (ms / 1000 / 60 / 60 / 24) % 31;
	uint8_t MM =   (ms / 1000 / 60 / 60 / 24 / 31) % 12;
	uint16_t YYYY = ms / 1000 / 60 / 60 / 24 / 31 / 12;
	
	// ISO 8601 time
	return sprintf_(str, "%04d-%02d-%02d %02d:%02d:%02d.%03d ",
		YYYY, MM, DD, hh, mm, ss, sss);
}
