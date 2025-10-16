/*
 * Copyright (C) 2025 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <array>
#include <fstream>
#include <iostream>
#include <string>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>
#include <gz/common/IOBase.hh>

#include <gz/math/TimeVaryingVolumetricGrid.hh>
#include <gz/math/TimeVaryingVolumetricGridLookupField.hh>
#include <gz/math/VolumetricGridLookupField.hh>

int main(int argc, char **argv)
{
    std::string dataFile{"../volumetric_grid_example.csv"};
    std::ifstream dataStream(dataFile);
    if (!dataStream.is_open())
    {
        std::cout << "No data file was found at " << dataFile << std::endl;
        return -1;
    }
    std::cout << "Loading data " << dataFile << std::endl;

    std::string timeColumnName{"timestamp"};
    std::array<std::string, 3> spatialColumnNames{"x", "y", "z"};

    using T = gz::math::InMemoryTimeVaryingVolumetricGrid<double>;
    using FrameT = gz::common::DataFrame<std::string, T>;

    auto dataFrame = gz::common::IO<FrameT>::ReadFrom(
        gz::common::CSVIStreamIterator(dataStream),
        gz::common::CSVIStreamIterator(),
        timeColumnName,
        spatialColumnNames);

    // check we have expected columns - note timestamp, x, y, z are not included.
    std::array<std::string, 7> columnNames{"timestamp", "x", "y", "z", "vx"};
    for (auto key: columnNames)
    {
        if (dataFrame.Has(key))
        {
            std::cout << "Have " << key << std::endl;
        }
        else
        {
            std::cout << "Missing " << key << std::endl;
        }
    }

    // get the data frame keys
    for (auto key: dataFrame.Keys())
    {
        std::cout << "Key " << key << std::endl;
    }

    // queries on columns
    std::string key{"vx"};
    auto dataColumn = dataFrame[key];
    auto session = dataColumn.CreateSession(0.0);

    // check valid
    if (dataColumn.IsValid(session))
    {
        std::cout << "Column for key " << key << " is valid." << std::endl;
    }
    else
    {
        std::cout << "Column for key " << key << " is not valid." << std::endl;
    }

    double vMax = 0.0;
    double vMin = 0.0;
    for (int i = 0; i < dataColumn.values.size(); ++i)
    {
        double v = dataColumn.values[i];
        vMax = std::max(vMax, v);
        vMin = std::min(vMin, v);
    }

    std::cout << "Values" << std::endl;
    std::cout << "nx: " << dataColumn.values.size() << std::endl;
    std::cout << "vMin: " << vMin << ", vMax: " << vMax << std::endl;

    // get bounds - these are the grid extents
    auto [lowerBound, upperBound] = dataColumn.Bounds(session);
    std::cout << "Bounds: [" << lowerBound << "], [" << upperBound << "]" << std::endl; 

    // get the range of values in the field
    int nx = 4;
    int ny = 4;
    int nz = 4;
    auto range = upperBound - lowerBound;
    auto dx = range.X() / nx;
    auto dy = range.Y() / ny;
    auto dz = range.Z() / nz;
    for (int ix = 0; ix < nx; ++ix)
    {
        auto x = lowerBound.X() + ix * dx;
        for (int iy = 0; iy < ny; ++iy)
        {
            auto y = lowerBound.Y() + iy * dy;
            for (int iz = 0; iz < nz; ++iz)
            {
                auto z = lowerBound.Z() + iz * dz;
                auto v = dataColumn.LookUp(session, gz::math::Vector3(x, y, z));
                if (v.has_value())
                {
                    vMax = std::max(vMax, v.value());
                    vMin = std::min(vMin, v.value());
                    std::cout
                        << "[" << ix << ", " << iy << ", " << iz << "], "
                        << "[" << x << ", " << y << ", " << z << "], "
                        << v.value() << std::endl;
                }
            }
        }
    }

    // known failure
    int ix = 1;
    auto x = lowerBound.X() + ix * dx;
    int iy = 1;
    auto y = lowerBound.Y() + iy * dy;
    int iz = 1;
    auto z = lowerBound.Z() + iz * dz;
    auto v = dataColumn.LookUp(session, gz::math::Vector3(x, y, z));
    std::cout << "Known Failure" << std::endl;
    std::cout
        << "[" << ix << ", " << iy << ", " << iz << "], "
        << "[" << x << ", " << y << ", " << z << "], "
        << v.value() << std::endl;
    std::cout << "-------------" << std::endl;

    std::cout << "Samples" << std::endl;
    std::cout << "nx: " << nx << ", ny: " << ny << ", nz: " << nz << std::endl;
    std::cout << "dx: " << dx << ", dy: " << dy << ", dz: " << dz << std::endl;
    std::cout << "vMin: " << vMin << ", vMax: " << vMax << std::endl;

    return 0;
}
