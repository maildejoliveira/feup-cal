//Taken from stackoverflow https://stackoverflow.com/questions/25201131/writing-csv-files-from-c/25202375

#pragma once

#include <iostream>
#include <fstream>
#define DATA_FILE_PATH "../src/data.csv"

class Csvfile;

inline static Csvfile& endrow(Csvfile& file);
inline static Csvfile& flush(Csvfile& file);

class Csvfile
{
    std::ofstream fs_;
    const std::string separator_;
public:
    Csvfile(const std::string filename, const std::string separator = ",")
            : fs_()
            , separator_(separator)
    {
        fs_.exceptions(std::ios::failbit | std::ios::badbit);
        fs_.open(filename, ios::out | ios::app);
    }

    ~Csvfile()
    {
        flush();
        fs_.close();
    }

    void flush()
    {
        fs_.flush();
    }

    void endrow()
    {
        fs_ << std::endl;
    }

    Csvfile& operator << (Csvfile& (* val)(Csvfile&))
    {
        return val(*this);
    }

    Csvfile& operator << (const char * val)
    {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    Csvfile& operator << (const std::string & val)
    {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    template<typename T>
    Csvfile& operator << (const T& val)
    {
        fs_ << val << separator_;
        return *this;
    }
};


inline static Csvfile& endrow(Csvfile& file)
{
    file.endrow();
    return file;
}

inline static Csvfile& flush(Csvfile& file)
{
    file.flush();
    return file;
}