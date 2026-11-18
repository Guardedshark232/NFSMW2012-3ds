#ifndef BNDL_H
#define BNDL_H

#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <stdio.h>

// Criterion "bnd2" container.
//
// Header (40 bytes used, the first 0x70 bytes are reserved for the header):
//   +0  char magic[4]      "bnd2"
//   +4  u32 version        0x00010005
//   +8  u32 fileSize
//   +12 u32 numEntries
//   +16 u32 entryOffset    (0x70 in every retail bundle seen)
//   +20 u32 dataOffset     (start of the stream-0 region)
//   +24 u32 bulkOffset     (start of the stream-1 / bulk region)
//   +28 u32 fileSize (copy)
//   +32 u32 fileSize (copy)
//   +36 u32 flags
//
// Entry table: numEntries records of exactly 72 bytes.
//   +0  u64 id
//   +8  u32 size0          uncompressed size of stream 0
//   +12 u32 size1          uncompressed size of stream 1
//   +24 u32 comp0          stored size of stream 0
//   +28 u32 comp1          stored size of stream 1
//   +40 u32 offset0        relative to header.dataOffset
//   +44 u32 offset1        relative to header.bulkOffset
//   +56 u32 payloadSize
//   +60 u32 typeId
//
// Size fields carry flags in their top nibble: 0x40000000 means the stream
// is stored uncompressed (raw), 0x70000000 is also seen for bulk data.
#pragma pack(push, 1)
struct BNDL2Header {
    char     magic[4];
    uint32_t version;
    uint32_t fileSize;
    uint32_t numEntries;
    uint32_t entryOffset;
    uint32_t dataOffset;
    uint32_t bulkOffset;
    uint32_t fileSize2;
    uint32_t fileSize3;
    uint32_t flags;
};

struct BNDL2RawEntry {
    uint64_t id;           // +0
    uint32_t size0;        // +8
    uint32_t size1;        // +12
    uint32_t reserved0;    // +16
    uint32_t reserved1;    // +20
    uint32_t comp0;        // +24
    uint32_t comp1;        // +28
    uint32_t reserved2;    // +32
    uint32_t reserved3;    // +36
    uint32_t offset0;      // +40  relative to dataOffset
    uint32_t offset1;      // +44  relative to bulkOffset
    uint32_t reserved4;    // +48
    uint32_t reserved5;    // +52
    uint32_t payloadSize;  // +56
    uint32_t typeId;       // +60
    uint32_t reserved6;    // +64
    uint32_t reserved7;    // +68
};
#pragma pack(pop)

static_assert(sizeof(BNDL2Header) == 40, "BNDL2Header must be 40 bytes");
static_assert(sizeof(BNDL2RawEntry) == 72, "BNDL2RawEntry must be 72 bytes");

static const uint32_t BNDL2_SIZE_MASK = 0x0FFFFFFF;
static const uint32_t BNDL2_FLAG_MASK = 0xF0000000;

// Resource type ids observed in the retail bundles.
enum BNDLResourceType {
    BNDL_TYPE_TEXTURE      = 1,
    BNDL_TYPE_RENDERABLE   = 5,
    BNDL_TYPE_TYPEINFO     = 20,
    BNDL_TYPE_INSTANCE     = 21,
    BNDL_TYPE_FONT         = 48,
};

// One decoded entry of the archive.
struct BNDLResource {
    uint64_t id;
    uint32_t typeId;

    uint32_t size0;         // declared uncompressed size of stream 0
    uint32_t comp0;         // stored size of stream 0
    uint32_t offset0;       // absolute file offset of stream 0
    bool     raw0;          // stream 0 stored uncompressed

    uint32_t size1;         // declared size of stream 1 (0 if absent)
    uint32_t comp1;         // stored size of stream 1
    uint32_t offset1;       // absolute file offset of stream 1
    bool     raw1;          // stream 1 stored uncompressed

    bool hasStream1() const { return comp1 != 0; }
};

// Header of a type-1 texture resource.  The pixel data lives in stream 1.
struct TextureInfo {
    uint32_t format;    // DXGI format id (71 = BC1, 77 = BC3, ...)
    uint16_t width;
    uint16_t height;
    uint32_t mips;
    uint32_t dataSize;
};

class BNDLArchive {
public:
    BNDLArchive();
    ~BNDLArchive();

    bool open(const char* path);
    void close();

    bool isOpen() const { return m_file != nullptr; }
    uint32_t getNumEntries() const { return (uint32_t)m_resources.size(); }
    const BNDLResource* getResource(uint32_t index) const;
    const BNDL2Header* getHeader() const { return &m_header; }

    int findById(uint64_t id) const;
    int findByType(uint32_t typeId, int start = 0) const;

    bool readRaw(uint32_t offset, void* out, uint32_t size);

    // Reads/inflates one stream. `outCap` is the capacity of `out`.
    bool readStream0(uint32_t index, void* out, uint32_t outCap, uint32_t* bytesWritten = nullptr);
    bool readStream1(uint32_t index, void* out, uint32_t outCap, uint32_t* bytesWritten = nullptr);

    // Parses the 48-byte texture descriptor stored in stream 0.
    bool readTextureInfo(uint32_t index, TextureInfo* out);

    // Allocates and reads a whole stream. Caller frees().
    void* loadStream0(uint32_t index, uint32_t* outSize);
    void* loadStream1(uint32_t index, uint32_t* outSize);

    static bool inspect(const char* path, BNDL2Header* outHeader);

private:
    bool readChunk(uint32_t offset, uint32_t storedSize, bool raw,
                   void* out, uint32_t outCap, uint32_t* bytesWritten);

    FILE* m_file;
    BNDL2Header m_header;
    std::vector<BNDLResource> m_resources;
};

#endif // BNDL_H
