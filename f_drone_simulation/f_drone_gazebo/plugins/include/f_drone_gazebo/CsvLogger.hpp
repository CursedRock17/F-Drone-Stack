#include <fstream>
#include <string>

// Simple Logging Class to Help w/CSV Export
class CsvLogger {
public:
    CsvLogger(const std::string & filename, const std::string & header = "")
    {
        file.open(filename, std::ios::out | std::ios::trunc);
        if (file.is_open() && !header.empty()) {
            file << header << "\n";
            file.flush();
        }
    }

    ~CsvLogger()
    {
        if (file.is_open()) {
            file.flush();
            file.close();
        }
    }

    template<typename... Args>
    void log(Args... args)
    {
        if (!file.is_open()) return;
        write(args...);
        file << "\n";
    }

    void flush()
    {
        if (file.is_open())
            file.flush();
    }

private:
    std::ofstream file;

    template<typename T>
    void write(const T &value)
    {
        file << value;
    }

    template<typename T, typename... Args>
    void write(const T &first, const Args&... rest)
    {
        file << first << ",";
        write(rest...);
    }
};
