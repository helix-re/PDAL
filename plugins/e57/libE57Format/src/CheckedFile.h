#ifndef CHECKED_FILE_P_H
#define CHECKED_FILE_P_H

/*
 * Copyright 2009 - 2010 Kevin Ackley (kackley@gwi.net)
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <algorithm>

#include "Common.h"

typedef long long off_t_ll;

namespace e57 {

   class CheckedFile
   {
      public:
         static constexpr size_t   physicalPageSizeLog2 = 10;  // physical page size is 2 raised to this power
         static constexpr size_t   physicalPageSize = 1 << physicalPageSizeLog2;
         static constexpr off_t_ll physicalPageSizeMask = physicalPageSize - 1;
         static constexpr size_t   logicalPageSize = physicalPageSize - 4;

      public:
         enum Mode
         {
            ReadOnly,
            WriteCreate,
            WriteExisting
         };

         enum OffsetMode
         {
            Logical,
            Physical
         };

         CheckedFile( const e57::ustring &fileName, Mode mode, ReadChecksumPolicy policy );
         ~CheckedFile();

         void            read(char* buf, size_t nRead, size_t bufSize = 0);
         void            write(const char* buf, size_t nWrite);
         CheckedFile&    operator<<(const e57::ustring& s);
         CheckedFile&    operator<<(int64_t i);
         CheckedFile&    operator<<(uint64_t i);
         CheckedFile&    operator<<(float f);
         CheckedFile&    operator<<(double d);
         void			 seek(off_t_ll offset, OffsetMode omode = Logical);
         off_t_ll		 position(OffsetMode omode = Logical);
         off_t_ll        length(OffsetMode omode = Logical);
         void            extend(off_t_ll newLength, OffsetMode omode = Logical);
         e57::ustring    fileName() const { return fileName_; }
         void            close();
         void            unlink();

         static inline off_t_ll logicalToPhysical(off_t_ll logicalOffset);
         static inline off_t_ll physicalToLogical(off_t_ll physicalOffset);

      private:
         uint32_t    checksum(char* buf, size_t size) const;
         void        verifyChecksum( char *page_buffer, size_t page );

         template<class FTYPE>
         CheckedFile&    writeFloatingPoint(FTYPE value, int precision);

         void        getCurrentPageAndOffset(off_t_ll& page, size_t& pageOffset, OffsetMode omode = Logical);
         void        readPhysicalPage(char* page_buffer, off_t_ll page);
         void        writePhysicalPage(char* page_buffer, off_t_ll page);
         int         portableOpen( const e57::ustring &fileName,
             int flags, int mode );
         off_t_ll portableSeek(off_t_ll offset, int whence);


         e57::ustring    fileName_;
         off_t_ll		 logicalLength_ = 0;
         off_t_ll		 physicalLength_ = 0;

         ReadChecksumPolicy checkSumPolicy_ = CHECKSUM_POLICY_ALL;

         int             fd_ = -1;
         bool            readOnly_ = false;
   };

   inline off_t_ll CheckedFile::logicalToPhysical(off_t_ll logicalOffset)
   {
       const off_t_ll page = logicalOffset / logicalPageSize;
       const off_t_ll remainder = logicalOffset - page * logicalPageSize;

      return page*physicalPageSize + remainder;
   }

   inline off_t_ll CheckedFile::physicalToLogical(off_t_ll physicalOffset)
   {
      const off_t_ll page = physicalOffset >> physicalPageSizeLog2;
      const size_t remainder = static_cast<size_t> (physicalOffset & physicalPageSizeMask);

      return page*logicalPageSize + (std::min)(remainder, logicalPageSize);
   }

}

#endif
