#include <stdlib.h>
#include "common/assert.h"
#include "nmea.hpp"
#include "third_party/libnmea/src/nmea/nmea.h"
#include "third_party/libnmea/src/parsers/gpgga.h"
#include "third_party/libnmea/src/parsers/gpgll.h"
#include "third_party/libnmea/src/parsers/gpgsv.h"
#include "third_party/libnmea/src/parsers/gprmc.h"

using namespace drv;

constexpr size_t neo7m_nmea_msg_number = 12;

nmea::nmea(hal::uart &uart, UBaseType_t task_priority):
	_uart(uart)
{
	ASSERT(xTaskCreate(this->task, "nmea", configMINIMAL_STACK_SIZE + 500, this,
		task_priority, &task_hndlr) == pdPASS);
	
	ASSERT(queue = xQueueCreate(neo7m_nmea_msg_number, sizeof(queue_t)));
}

nmea::~nmea()
{
	vTaskDelete(task_hndlr);
	vQueueDelete(queue);
}

void nmea::task(void *pvParameters)
{
	nmea *obj = (nmea *)pvParameters;
	
	while(1)
	{
		char buff[NMEA_MAX_LENGTH * neo7m_nmea_msg_number] = {};
		uint16_t size = sizeof(buff);
		
		int8_t res = obj->_uart.read((uint8_t *)buff, &size, portMAX_DELAY);
		if(res != hal::uart::RES_OK)
		{
			// TODO: Indicate application about uart error
			continue;
		}
		buff[size] = '\0';
		
		obj->on_recv((char *)buff, size);
	}
}

void nmea::on_recv(char *buff, size_t size)
{
	char *line = NULL;
	size_t line_size;
	
	while(get_line(buff, size, &line, &line_size))
	{
		queue_t queue_data = {.res = RES_OK};
		nmea_s *data = nmea_parse(line, line_size, 1);
		if(!data)
		{
			queue_data.res = RES_PARSE_ERR;
			xQueueSend(queue, &queue_data, 0);
			continue;
		}
		
		queue_data = format_queue(data);
		xQueueSend(queue, &queue_data, 0);
		nmea_free(data);
	}
}

nmea::queue_t nmea::format_queue(nmea_s *data)
{
	queue_t queue_data = {.res = RES_OK};
	queue_data.type = data->type;
	
	switch(data->type)
	{
		case NMEA_GPGGA: queue_data.gpgga = *((nmea_gpgga_s *)data); break;
		case NMEA_GPGLL: queue_data.gpgll = *((nmea_gpgll_s *)data); break;
		case NMEA_GPRMC: queue_data.gprmc = *((nmea_gprmc_s *)data); break;
		case NMEA_GPGSV: queue_data.gpgsv = *((nmea_gpgsv_s *)data); break;
		default: queue_data.res = RES_UNKNOWN_MSG_ERR;
	}
	
	return queue_data;
}

bool nmea::get_line(char *buff, size_t size, char **line, size_t *line_size)
{
	if(!*line)
	{
		*line = buff;
		*line_size = 0;
	}
	else
		*line+= *line_size;
	
	size_t remain = size - (*line - buff);
	if(remain == 0)
		return false;
	
	char *start = (char *)memchr(*line, '$', remain);
	if(!start)
	{
		if(!(start = (char *)memchr(*line, '!', remain)))
			return false;
	}
	remain -= (size_t)start - (size_t)*line;
	
	char *end = (char *)memchr(start, '\r', remain);
	if(!end || &end[1] > &start[remain] || end[1] != '\n')
		return false;
	
	*line = start;
	*line_size = end + sizeof("\r\n") - 1 - start;
	
	return true;
}
