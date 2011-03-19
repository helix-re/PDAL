/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <cassert>
#include <libpc/exceptions.hpp>
#include <libpc/Color.hpp>
#include <libpc/filters/ColorFilter.hpp>
#include <libpc/filters/ColorFilterIterator.hpp>

namespace libpc { namespace filters {

ColorFilter::ColorFilter(Stage& prevStage)
    : Filter(prevStage)
{
    checkImpedance();

    return;
}


void ColorFilter::checkImpedance()
{
    Schema& schema = getHeader().getSchema();

    Dimension dimZ(Dimension::Field_Z, Dimension::Uint8);
    if (schema.hasDimension(dimZ) == false)
    {
        throw impedance_invalid("color filter does not have Z/uint8 field");
    }

    Dimension dimRed(Dimension::Field_Red, Dimension::Uint8);     
    Dimension dimGreen(Dimension::Field_Green, Dimension::Uint8);
    Dimension dimBlue(Dimension::Field_Blue, Dimension::Uint8);

    // are there already u8 fields for color?
    if (!schema.hasDimension(dimRed))
    {
        schema.addDimension(dimRed);
    }
    if (!schema.hasDimension(dimGreen))
    {
        schema.addDimension(dimGreen);
    }
    if (!schema.hasDimension(dimBlue))
    {
        schema.addDimension(dimBlue);
    }

    return;
}


const std::string& ColorFilter::getName() const
{
    static std::string name("Color Filter");
    return name;
}


void ColorFilter::getColor(float value, boost::uint8_t& red, boost::uint8_t& green, boost::uint8_t& blue) const
{
    double fred, fgreen, fblue;

    const Range<double>& zrange = getHeader().getBounds().dimensions()[2];
    Color::interpolateColor(value, zrange.getMinimum(), zrange.getMaximum(), fred, fblue, fgreen);

    const double vmax = (std::numeric_limits<boost::uint8_t>::max());
    red = (boost::uint8_t)(fred * vmax);
    green = (boost::uint8_t)(fgreen * vmax);
    blue = (boost::uint8_t)(fblue * vmax);

    return;
}


libpc::Iterator* ColorFilter::createIterator()
{
    return new ColorFilterIterator(*this);
}


} } // namespaces
