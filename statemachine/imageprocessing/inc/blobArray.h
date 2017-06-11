#ifndef BLOBARRAY_H
#define BLOBARRAY_H

#include "blob.h"

class BlobArray {
public:
    BlobArray() {}
    std::vector<Blob> blobs;
    /**
     * @brief getClosestBlob from blob array
     * @return closest blob
     */
    Blob getClosestBlob();
};


#endif //BLOBARRAY_H
