#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <vector>
#include "FreeRTOS.h"
#include "semphr.h"

// Macro for easy and readable use of the system log
#define log() ul::syslog::get_instance()

// Max string size including timestamp, tag and end of line (\r\n)
#define MAX_STRING_SIZE 200

namespace ul
{
class syslog
{
	public:
		static syslog &get_instance()
		{
			static syslog instance;
			return instance;
		}
		
		typedef void (*cb_t)(uint8_t *buff, size_t size, void *ctx);
		void add_output(cb_t cb, void *ctx = NULL);
		void del_output(cb_t cb, void *ctx = NULL);
		
		// Do not call APIs below from ISR
		
		// Debug messages that we only rarely turn on
		void dbg(const char *format, ...);
		
		/* Anything that we want to know when looking at the log files, e.g.
		when a scheduled job started/ended */
		void inf(const char *format, ...);
		
		/* Any messages that might warn us of potential problems, e.g. when a
		user tried to log in with wrong credentials - which might indicate an
		attack if that happens often or in short period of time */
		void wrn(const char *format, ...);
		
		// Any error that is or might be critical
		void err(const char *format, ...);
		
		syslog(syslog const &) = delete;
		void operator =(syslog const &) = delete;
		
	private:
		syslog();
		//syslog(syslog const &);
		//void operator =(syslog const &);
		
		static syslog *instance;
		SemaphoreHandle_t api_lock;
		
		struct cb_ctx_t
		{
			cb_t cb;
			void *ctx;
			
			// Overload "==" operator to use .find() methon of the list
			bool operator==(const cb_ctx_t &a) const
			{
				return (cb == a.cb && ctx == a.ctx);
			}
		};
		std::vector<cb_ctx_t>cb_ctx_list;
		
		enum tag_t {DBG, INF, WRN, ERR};
		static constexpr const char *tags[] = {"(dbg) ", "(inf) ", "(wrn) ",
			"(err) "};
		char message[MAX_STRING_SIZE];
		void print(tag_t tag, const char *format, va_list va);
		size_t add_tag(char *str, uint8_t tag);
		size_t add_eol(char *str);
		size_t add_timestamp(char *str);
};
}
