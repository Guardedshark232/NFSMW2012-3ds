#include "bndl.h"
#include "tinf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

BNDLArchive::BNDLArchive() : m_file(nullptr) {
    memset(&m_header, 0, sizeof(m_header));
}

BNDLArchive::~BNDLArchive() {
    close();
}

bool BNDLArchive::inspect(const char* path, BNDL2Header* outHeader) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    BNDL2Header header;
    if (fread(&header, sizeof(header), 1, f) != 1) {
        fclose(f);
        return false;
    }
    fclose(f);

    if (memcmp(header.magic, "bnd2", 4) != 0) {
        return false;
    }

    if (outHeader) {
        *outHeader = header;
    }
    return true;
}

bool BNDLArchive::open(const char* path) {
    close();

    m_file = fopen(path, "rb");
    if (!m_file) return false;

    if (fread(&m_header, sizeof(m_header), 1, m_file) != 1) {
        close();
        return false;
    }

    if (memcmp(m_header.magic, "bnd2", 4) != 0) {
        close();
        return false;
    }

    if (m_header.numEntries == 0 || m_header.numEntries > 200000) {
        close();
        return false;
    }

    if (fseek(m_file, (long)m_header.entryOffset, SEEK_SET) != 0) {
        close();
        return false;
    }

    m_resources.clear();
    m_resources.reserve(m_header.numEntries);

    for (uint32_t i = 0; i < m_header.numEntries; i++) {
        BNDL2RawEntry raw;
        if (fread(&raw, sizeof(raw), 1, m_file) != 1) {
            break;
        }

        BNDLResource res;
        res.id = raw.id;
        res.typeId = raw.typeId;

        res.size0 = raw.size0 & BNDL2_SIZE_MASK;
        res.comp0 = raw.comp0 & BNDL2_SIZE_MASK;
        res.raw0 = (raw.comp0 & BNDL2_FLAG_MASK) != 0;
        res.offset0 = m_header.dataOffset + raw.offset0;

        res.size1 = raw.size1 & BNDL2_SIZE_MASK;
        res.comp1 = raw.comp1 & BNDL2_SIZE_MASK;
        res.raw1 = (raw.comp1 & BNDL2_FLAG_MASK) != 0;
        res.offset1 = m_header.bulkOffset + raw.offset1;

        m_resources.push_back(res);
    }

    return true;
}

void BNDLArchive::close() {
    if (m_file) {
        fclose(m_file);
        m_file = nullptr;
    }
    m_resources.clear();
}

const BNDLResource* BNDLArchive::getResource(uint32_t index) const {
    if (index >= m_resources.size()) return nullptr;
    return &m_resources[index];
}

int BNDLArchive::findById(uint64_t id) const {
    for (size_t i = 0; i < m_resources.size(); i++) {
        if (m_resources[i].id == id) return (int)i;
    }
    return -1;
}

int BNDLArchive::findByType(uint32_t typeId, int start) const {
    if (start < 0) start = 0;
    for (size_t i = (size_t)start; i < m_resources.size(); i++) {
        if (m_resources[i].typeId == typeId) return (int)i;
    }
    return -1;
}

bool BNDLArchive::readRaw(uint32_t offset, void* out, uint32_t size) {
    if (!m_file || !out) return false;
    if (fseek(m_file, (long)offset, SEEK_SET) != 0) return false;
    return fread(out, 1, size, m_file) == size;
}

bool BNDLArchive::readChunk(uint32_t offset, uint32_t storedSize, bool raw,
                            void* out, uint32_t outCap, uint32_t* bytesWritten) {
    if (!m_file || !out || storedSize == 0) return false;

    if (raw) {
        if (storedSize > outCap) return false;
        if (!readRaw(offset, out, storedSize)) return false;
        if (bytesWritten) *bytesWritten = storedSize;
        return true;
    }

    if (storedSize > (64u << 20)) return false; // sanity

    void* comp = malloc(storedSize);
    if (!comp) return false;

    bool ok = readRaw(offset, comp, storedSize);
    if (!ok) {
        free(comp);
        return false;
    }

    unsigned int destLen = outCap;
    const uint8_t* b = (const uint8_t*)comp;
    int rc;
    if (storedSize >= 2 && b[0] == 0x78) {
        rc = tinf_zlib_uncompress(out, &destLen, comp, storedSize);
    } else {
        rc = tinf_uncompress(out, &destLen, comp, storedSize);
    }
    free(comp);

    if (rc != TINF_OK) return false;

    if (bytesWritten) *bytesWritten = destLen;
    return true;
}

bool BNDLArchive::readStream0(uint32_t index, void* out, uint32_t outCap, uint32_t* bytesWritten) {
    const BNDLResource* res = getResource(index);
    if (!res) return false;
    return readChunk(res->offset0, res->comp0, res->raw0, out, outCap, bytesWritten);
}

bool BNDLArchive::readStream1(uint32_t index, void* out, uint32_t outCap, uint32_t* bytesWritten) {
    const BNDLResource* res = getResource(index);
    if (!res || !res->hasStream1()) return false;
    return readChunk(res->offset1, res->comp1, res->raw1, out, outCap, bytesWritten);
}

bool BNDLArchive::readTextureInfo(uint32_t index, TextureInfo* out) {
    const BNDLResource* res = getResource(index);
    if (!res || !out) return false;

    uint8_t hdr[64];
    uint32_t got = 0;
    if (!readStream0(index, hdr, sizeof(hdr), &got)) return false;
    if (got < 48) return false;

    // Layout (little endian):
    //   +28 u32 DXGI format
    //   +32 u32 mip/array count
    //   +36 u16 width
    //   +38 u16 height
    uint32_t format, mips;
    uint16_t w, h;
    memcpy(&format, hdr + 28, 4);
    memcpy(&mips,   hdr + 32, 4);
    memcpy(&w,      hdr + 36, 2);
    memcpy(&h,      hdr + 38, 2);

    out->format = format;
    out->mips = mips;
    out->width = w;
    out->height = h;
    out->dataSize = res->size1;
    return w != 0 && h != 0;
}

void* BNDLArchive::loadStream0(uint32_t index, uint32_t* outSize) {
    const BNDLResource* res = getResource(index);
    if (!res) return nullptr;

    // Inflated length can slightly exceed the declared size, so add slack.
    uint32_t cap = res->size0 + 4096;
    void* buf = malloc(cap);
    if (!buf) return nullptr;

    uint32_t got = 0;
    if (!readStream0(index, buf, cap, &got)) {
        free(buf);
        return nullptr;
    }
    if (outSize) *outSize = got;
    return buf;
}

void* BNDLArchive::loadStream1(uint32_t index, uint32_t* outSize) {
    const BNDLResource* res = getResource(index);
    if (!res || !res->hasStream1()) return nullptr;

    uint32_t cap = res->size1 + 65536;
    void* buf = malloc(cap);
    if (!buf) return nullptr;

    uint32_t got = 0;
    if (!readStream1(index, buf, cap, &got)) {
        free(buf);
        return nullptr;
    }
    if (outSize) *outSize = got;
    return buf;
}
