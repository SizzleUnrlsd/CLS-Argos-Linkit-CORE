#ifndef __SENSOR_LOG_HPP_
#define __SENSOR_LOG_HPP_

#include "logger.hpp"
#include "filesystem.hpp"

class SensorLog : public Logger {
public:
	static const unsigned int MAX_SIZE = 4 * 1024 * 1024;
	static const char *filename = "sensor.log";

	SensorLog(FileSystem *fs) {
		m_filesystem = fs;
		CircularFile *f;
		try {
			// See if an existing file exists
			f = new CircularFile(m_filesystem, SensorLog::filename, LFS_O_RDONLY, SensorLog::MAX_SIZE);
		} catch (int e) {
			// Create a new log file if the file does not exist
			f = new CircularFile(m_filesystem, SensorLog::filename, LFS_O_WRONLY | LFS_O_CREAT, SensorLog::MAX_SIZE);
		}
		delete f;
	}

	void write(void *) {
		throw "not permitted";
	}

	void read(void *) {
		throw "not permitted";
	}

	void write(GPSLogEntry *entry) {
		CircularFile f(m_filesystem, SensorLog::filename, LFS_O_WRONLY, SensorLog::MAX_SIZE);
		f.write(entry, (lfs_size_t)sizeof(*entry));
	}

	void read(GPSLogEntry *entry, int index=0) {
		CircularFile f(m_filesystem, SensorLog::filename, LFS_O_RDONLY, SensorLog::MAX_SIZE);
		if (index != 0)
			f.seek(f.m_offset + index * (sizeof(*entry)));
		f.read(entry, (lfs_size_t)sizeof(*entry));
	}

	unsigned int num_entries() {
		CircularFile f(m_filesystem, "sensor.log", LFS_O_RDONLY, SensorLog::MAX_SIZE);
		return f.size() / sizeof(GPSLogEntry);
	}

private:
	FileSystem *m_filesystem;
};

#endif // __SENSOR_LOG_HPP_
