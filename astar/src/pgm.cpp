#include <string>
#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "pgm.hpp"

using namespace std;

PGM::PGM(const string &path)
{
    if (!isPGM(path))
    {
    }

    ifstream fin;
    if ()
    {
        // read file head
        fin >> ch >> this->type;
        fin >> this->width >> this->height;
        fin >> this->maxGrayScale;

        // allocate memory for data
        int len = this->width * this->height;
        this->mat = new char[len];

        // read binary data
        fin.read(&ch, 1); // read \n
        fin.read(this->mat, len);
    }
    else
    {
    }
}

~PGM()
{
    delete[] this->mat;
}

bool isPGM(const string &path) const
{
    ifstream fin;
    fin.open(path, ios::in);

    if (!fin.is_open())
    {
        cout << "Can't open file: \"" << path << "\"" << endl;
        return false;
    }
    else
    {
        char ch;
        int type;
        fin >> ch >> type;

        if ('P' == ch && (2 == type || 5 == type))
        {
            return true;
        }
        else
        {
            return false;
        }
        return isPGM(path);
    }
}

void show()
{
    return;
}