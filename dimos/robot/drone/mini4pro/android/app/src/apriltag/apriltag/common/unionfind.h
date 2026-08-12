/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#pragma once

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct unionfind unionfind_t;

// Interleave node parent and size to improve cache locality.
// Accessing parent[i] and size[i] hits the same cache line.
struct unionfind_node
{
    // Parent node for each. Initialized to 0xffffffff
    uint32_t parent;
    // The size of the tree excluding the root
    uint32_t size;
};

struct unionfind
{
    uint32_t maxid;
    struct unionfind_node *data;
};

static inline uint32_t uf_load_parent(unionfind_t *uf, uint32_t id)
{
#if defined(__GNUC__) || defined(__clang__)
    return __atomic_load_n(&uf->data[id].parent, __ATOMIC_RELAXED);
#else
    return uf->data[id].parent;
#endif
}

static inline void uf_store_parent(unionfind_t *uf, uint32_t id, uint32_t val)
{
#if defined(__GNUC__) || defined(__clang__)
    __atomic_store_n(&uf->data[id].parent, val, __ATOMIC_RELAXED);
#else
    uf->data[id].parent = val;
#endif
}

static inline unionfind_t *unionfind_create(uint32_t maxid)
{
    unionfind_t *uf = (unionfind_t*) calloc(1, sizeof(unionfind_t));
    uf->maxid = maxid;
    uf->data = (struct unionfind_node *) malloc((maxid+1) * sizeof(struct unionfind_node));
    for (uint32_t i = 0; i <= maxid; i++) {
        uf->data[i].parent = 0xffffffff;
        uf->data[i].size = 0;
    }
    return uf;
}

static inline void unionfind_destroy(unionfind_t *uf)
{
    free(uf->data);
    free(uf);
}

/*
static inline uint32_t unionfind_get_representative(unionfind_t *uf, uint32_t id)
{
    // base case: a node is its own parent
    if (uf->parent[id] == id)
        return id;

    // otherwise, recurse
    uint32_t root = unionfind_get_representative(uf, uf->parent[id]);

    // short circuit the path. [XXX This write prevents tail recursion]
    uf->parent[id] = root;

    return root;
}
*/

// this one seems to be every-so-slightly faster than the recursive
// version above.
static inline uint32_t unionfind_get_representative(unionfind_t *uf, uint32_t id)
{
    // unititialized node, so set to self
    if (uf_load_parent(uf, id) == 0xffffffff) {
        uf_store_parent(uf, id, id);
        return id;
    }

    // Path halving: make every node point to its grandparent (single pass)
    while (uf_load_parent(uf, id) != id) {
        uf_store_parent(uf, id, uf_load_parent(uf, uf_load_parent(uf, id)));
        id = uf_load_parent(uf, id);
    }

    return id;
}

static inline uint32_t unionfind_get_set_size(unionfind_t *uf, uint32_t id)
{
    uint32_t repid = unionfind_get_representative(uf, id);
    return uf->data[repid].size + 1;
}

static inline uint32_t unionfind_connect(unionfind_t *uf, uint32_t aid, uint32_t bid)
{
    uint32_t aroot = unionfind_get_representative(uf, aid);
    uint32_t broot = unionfind_get_representative(uf, bid);

    if (aroot == broot)
        return aroot;

    // we don't perform "union by rank", but we perform a similar
    // operation (but probably without the same asymptotic guarantee):
    // We join trees based on the number of *elements* (as opposed to
    // rank) contained within each tree. I.e., we use size as a proxy
    // for rank.  In my testing, it's often *faster* to use size than
    // rank, perhaps because the rank of the tree isn't that critical
    // if there are very few nodes in it.
    uint32_t asize = uf->data[aroot].size + 1;
    uint32_t bsize = uf->data[broot].size + 1;

    // optimization idea: We could shortcut some or all of the tree
    // that is grafted onto the other tree. Pro: those nodes were just
    // read and so are probably in cache. Con: it might end up being
    // wasted effort -- the tree might be grafted onto another tree in
    // a moment!
    if (asize > bsize) {
        uf->data[broot].parent = aroot;
        uf->data[aroot].size += bsize;
        return aroot;
    } else {
        uf->data[aroot].parent = broot;
        uf->data[broot].size += asize;
        return broot;
    }
}
