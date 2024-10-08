#include "ImuLoader.h"
#include "csv.hpp"
#include <fstream>
#include <iostream>

namespace mandeye
{
    std::vector<std::tuple<double, ImuAngularVelocity, ImuAcceleration>> load_imu(const std::string& imu_file, int imuToUse)
    {
        std::vector<std::tuple<double, ImuAngularVelocity, ImuAcceleration>> all_data;

        csv::CSVFormat format;
        format.delimiter({ ' ', ',', '\t' });

        csv::CSVReader reader(imu_file, format);
        const auto columns = reader.get_col_names();
        const std::set<std::string> columnsSet(columns.begin(), columns.end());

        auto contains = [&columnsSet](const std::string& column) -> bool
        {
            return columnsSet.find(column) != columnsSet.end();
        };
        // mandatory columns
        const bool hasTsColumn = contains("timestamp");
        const bool hasGyrosColumns = contains("gyroX") && contains("gyroY") && contains("gyroZ");
        const bool hasAccsColumns = contains("accX") && contains("accY") && contains("accZ");

        // optional
        const bool hasImuIdColumn = contains("imuId");
        const bool hasUnixTimestampColumn = contains("timestampUnix");

        // check if legacy
        bool is_legacy = true;
        if (hasTsColumn)
        {
            is_legacy = false;
            if (!hasAccsColumns && !hasGyrosColumns)
            {
                std::cerr << "Input csv file is missing one of the mandatory columns :\n";
                std::cerr << "timestamp,gyroX,gyroY,gyroZ,accX,accY,accZ";
                return all_data;
            }
        }

        if (!is_legacy)
        {
            // check if all needed columns are in csv
            for (auto row : reader)
            {
                int imu_id = -1;
                if (hasImuIdColumn)
                {
                    int imu_id = row["imuId"].get<int>();
                }
                if (imu_id < 0 || imuToUse == imu_id)
                {
                    double timestamp = row["timestamp"].get<double>();
                    double timestampUnix = row["timestampUnix"].get<double>();
                    ImuAngularVelocity gyr;
                    gyr[0] = row["gyroX"].get<double>();
                    gyr[1] = row["gyroY"].get<double>();
                    gyr[2] = row["gyroZ"].get<double>();
                    ImuAcceleration acc;
                    acc[0] = row["accX"].get<double>();
                    acc[1] = row["accY"].get<double>();
                    acc[2] = row["accZ"].get<double>();
                    all_data.emplace_back(timestamp / 1e9, gyr, acc);
                }
            }
        }
        if (is_legacy)
        {
            std::ifstream myfile(imu_file);
            if (myfile.is_open())
            {
                while (myfile)
                {
                    double data[7];
                    double timestampUnix = 0.0;
                    int imuId = 0;
                    std::string line;
                    std::getline(myfile, line);
                    std::istringstream iss(line);
                    iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
                    if (!iss.eof())
                    {
                        iss >> imuId;
                    }
                    if (!iss.eof())
                    {
                        iss >> timestampUnix;
                    }
                    // std::cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " "
                    // << data[6] << std::endl;
                    if (data[0] > 0 && imuId == imuToUse)
                    {
                        ImuAngularVelocity gyr;
                        gyr[0] = data[1];
                        gyr[1] = data[2];
                        gyr[2] = data[3];

                        ImuAcceleration acc;
                        acc[0] = data[4];
                        acc[1] = data[5];
                        acc[2] = data[6];

                        all_data.emplace_back(data[0] / 1e9, gyr, acc);
                    }
                }
                myfile.close();
            }
        }
        return all_data;
    }
} // namespace mandeye