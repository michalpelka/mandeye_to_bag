#include "LasLoader.h"
#include <iostream>
std::vector<mandeye::Point> mandeye::load(const std::string &lazFile) {
    std::vector<Point> points;
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader)) {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed)) {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", lazFile.c_str());
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;
    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header)) {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }
    fprintf(stderr, "file '%s' contains %u points\n", lazFile.c_str(), header->number_of_point_records);
    laszip_point *point;
    if (laszip_get_point_pointer(laszip_reader, &point)) {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }


    for (int j = 0; j < header->number_of_point_records; j++) {
        if (laszip_read_point(laszip_reader)) {
            fprintf(stderr, "DLL ERROR: reading point %u\n", j);
            std::abort();
        }

        Point p;
        p.point.x() = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
        p.point.y() = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
        p.point.z() = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;

        points.emplace_back(p);

    }

    return points;
}