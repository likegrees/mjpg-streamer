/*
 * Copyright (C) 2017 by Daniel Clouse.
 *
 * This file is dual licensed: you can use it either under the terms of
 * the GPL, or the BSD license, at your option.
 *
 *  a) This library is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of the
 *     License, or (at your option) any later version.
 *
 *     This library is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public
 *     License along with this library; if not, write to the Free
 *     Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 *     MA 02110-1301 USA
 *
 * Alternatively,
 *
 *  b) Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *     1. Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *     2. Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *     CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *     INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *     CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *     SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *     NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *     OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *     EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h> /* DEBUG */
#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include "detect_color_blobs.h"


/**** Operations on Blob_Stats ******/

static void stats_init(Blob_Stats* stats_ptr,
                       unsigned short row,
                       unsigned short col_low,
                       unsigned short col_high) {
    stats_ptr->min_x = col_low;
    stats_ptr->max_x = col_high - 1;
    stats_ptr->min_y = row;
    stats_ptr->max_y = row;
    unsigned long count = col_high - col_low;
    stats_ptr->sum_x = count * (col_low + col_high - 1) / 2;
    stats_ptr->sum_y = row * count;
    stats_ptr->count = count;
}

static void stats_add_singleton(Blob_Stats* stats_ptr,
                                unsigned short row,
                                unsigned short col_low,
                                unsigned short col_high) {
    if (col_low < stats_ptr->min_x) stats_ptr->min_x = col_low;
    unsigned short high = col_high - 1;
    if (high > stats_ptr->max_x) stats_ptr->max_x = high;
    if (row < stats_ptr->min_y) stats_ptr->min_y = row;
    if (row > stats_ptr->min_y) stats_ptr->max_y = row;
    unsigned long count = col_high - col_low;
    stats_ptr->count += count;
}

// a = a + b
static void stats_add(Blob_Stats* a_ptr,
                      const Blob_Stats* b_ptr) {
    if (b_ptr->min_x < a_ptr->min_x) a_ptr->min_x = b_ptr->min_x;
    if (b_ptr->max_x > a_ptr->max_x) a_ptr->max_x = b_ptr->max_x;
    if (b_ptr->min_y < a_ptr->min_y) a_ptr->min_y = b_ptr->min_y;
    if (b_ptr->max_y > a_ptr->max_y) a_ptr->max_y = b_ptr->max_y;
    a_ptr->sum_x += b_ptr->sum_x;
    a_ptr->sum_y += b_ptr->sum_y;
    a_ptr->count += b_ptr->count;
}

/***** Operations on Uv_Runs ******/

typedef struct {
    unsigned short run_low;
    unsigned short run_high; // one past
} Uv_Run;

/* Return true if pixel jj within the given uv_row has both u and v values
   within threshold. */
static inline bool uv_is_in(int pixels,
                            const unsigned char uv_row[],
                            unsigned char u_low,
                            unsigned char u_high,
                            unsigned char v_low,
                            unsigned char v_high,
                            int jj) {
    const int uv_offset = jj / 2;
    unsigned char u = uv_row[uv_offset];
    unsigned char v = uv_row[(pixels / 4) + uv_offset];
    return (u >= u_low && u <= u_high && v >= v_low && v <= v_high);
}

/* Run through a single row of u and v values; for any pixel that is within
   threshold for both u and v, add it to a new uv_run entry. */
static unsigned int create_uv_run_sequence(int cols,
                                           int rows,
                                           unsigned char uv_row[],
                                           unsigned char u_low,
                                           unsigned char u_high,
                                           unsigned char v_low,
                                           unsigned char v_high,
                                           Uv_Run uv_run[]) {
    int uv_run_count = 0;
    const int pixels = cols * rows;
    int jj = 0;
    while (jj < cols) {
        if (uv_is_in(pixels, uv_row, u_low, u_high, v_low, v_high, jj)) {
            /* Start new run. */
            uv_run[uv_run_count].run_low = jj;
            jj += 2;
            while (jj < cols &&
                   uv_is_in(pixels, uv_row, u_low, u_high, v_low, v_high, jj)) {
                jj += 2;
            }
            /* Stop the run. */
            uv_run[uv_run_count].run_high = jj - 1;
            ++uv_run_count;
        } else {
            jj += 2;
        }
    }
    return uv_run_count;
}

/***** Operations on Yuv_Runs ******/

typedef struct {
    Blob_Set_Index parent_index;
    unsigned short run_low;
    unsigned short run_high; // one past
} Yuv_Run;


/**
 * @brief Start a new blob consisting of a single Blob_Set_Entry.
 *
 * @param p [in,out]       Pointer to the Blob_List.  The new blob is added to
 *                         this list.
 * @param yuv_run_ptr [in] The pixel column run that defines the contents of
 *                         the new entry.
 * @param row [in]         The row in which all the pixels lie.
 * @return The index of the new entry.
 */
static Blob_Set_Index make_set(Blob_List* p,
                               const Yuv_Run* yuv_run_ptr,
                               unsigned short row) {
    if (p->used_blob_set_count >= p->max_blob_set_count ||
        p->used_root_list_count >= p->max_root_list_count) {
        // No space for a new set.
        return 0;
    }

    /* Create a new root_list entry. */

    Blob_Set_Index new_root_list_index = p->used_root_list_count++;

    /* Create and initialize a new blob_set entry. */

    Blob_Set_Entry* new_entry_ptr = &p->blob_set[p->used_blob_set_count];
    new_entry_ptr->root_list_index = new_root_list_index;
    new_entry_ptr->parent_index = p->used_blob_set_count; // self pointer
    new_entry_ptr->rank = 0;
    stats_init(&new_entry_ptr->stats, row, yuv_run_ptr->run_low,
               yuv_run_ptr->run_high);

    /* Add the new Blob_Set_Entry to the root_list. */

    p->root_list[new_root_list_index] = new_entry_ptr->parent_index;
    ++p->used_blob_set_count;
    return new_entry_ptr->parent_index;
}


/**
 * @brief Delete an entry from the root_list.
 *
 * @param p [in,out]             Pointer to the Blob_List containing the
 *                               root_list.
 * @param delete_this_entry [in] The value of the entry to delete.
 */
static void delete_root_list_entry(Blob_List* p, 
                                   Blob_Set_Index delete_this_entry) {
    /* Find the root_list entry with value delete_this_entry. */

    Root_List_Index ix = p->blob_set[delete_this_entry].root_list_index;
    assert(ix < p->used_root_list_count);
    assert(p->root_list[ix] == delete_this_entry);

    /* Overwrite it with the last entry. */

    Root_List_Index last_entry = --p->used_root_list_count;
    Blob_Set_Index bx = p->root_list[last_entry];
    p->root_list[ix] = bx;
    p->blob_set[bx].root_list_index = ix;
}


/**
 * @brief Set the parent pointer of the child_indexTH blob_set to
 *        new_parent_index.
 *
 * @param p [in,out]            Pointer to the Blob_List containing blob_set.
 * @param child_index [in]      Index of the child entry within blob_set.
 * @param new_parent_index [in] Index of the new parent entry within blob_set.
 */
static void set_parent(Blob_List* p,
                       Blob_Set_Index child_index,
                       Blob_Set_Index new_parent_index) {
    //fprintf(stderr, "set_parent of %d to %d ", child_index, new_parent_index); //DEBUG
    assert(child_index < p->used_blob_set_count);
    assert(new_parent_index < p->used_blob_set_count);
    Blob_Set_Entry* child_ptr = &p->blob_set[child_index];
    child_ptr->parent_index = new_parent_index;
    stats_add(&p->blob_set[new_parent_index].stats, &child_ptr->stats);
    delete_root_list_entry(p, child_index);
}


/**
 * @brief Return the root of the blob which contains the given blob_set index.
 *
 * @param p [in,out]      Pointer to the Blob_List containing blob_set.
 * @param index [in]      Index of the entry of interest within blob_set.
 * @return The index of the root.
 */
static Blob_Set_Index find_root(Blob_List* p, Blob_Set_Index index) {
    Blob_Set_Index parent = index;
    assert(index < p->used_blob_set_count);
    //fprintf(stderr, "find_root of %d ", index); //DEBUG
    if (p->blob_set[index].parent_index != index) {
        // Note recursive call to find_root().
        parent = find_root(p, p->blob_set[index].parent_index);
        set_parent(p, index, parent);
    }
    //fprintf(stderr, "is %d\n", parent); //DEBUG
    return parent;
}

/**
 * @brief Link together the blobs rooted at s_index and t_index into a single
 *        blob.
 *
 * @param p [in,out]      Pointer to the Blob_List containing all blob sets.
 * @param s_index [in]    The index (into blob_set) of the root of some blob.
 * @param t_index [in]    The index of the root of some other blob.
 * @return The index (either s_index or t_index) chosen as the root.
 */
static Blob_Set_Index link(Blob_List* p,
                           Blob_Set_Index s_index,
                           Blob_Set_Index t_index) {
    assert(s_index > 0);
    assert(s_index < p->used_blob_set_count);
    assert(t_index > 0);
    assert(t_index < p->used_blob_set_count);
    unsigned short s_rank = p->blob_set[s_index].rank;
    unsigned short t_rank = p->blob_set[t_index].rank;
    if (s_rank > t_rank) {
        set_parent(p, t_index, s_index);
        return s_index;
    } else {
        if (s_rank == t_rank) {
            ++t_rank;
            p->blob_set[t_index].rank = t_rank;
            //if (t_index != 0) p->blob_set[t_index].rank = t_rank;
        }
        set_parent(p, s_index, t_index);
        return t_index;
    }
}


// neither a nor b is assumed to be singleton
static void yuv_run_union(Blob_List* p,
                          unsigned int a_row,
                          Yuv_Run* a,
                          unsigned int b_row,
                          Yuv_Run* b) {
    Blob_Set_Index a_parent = a->parent_index;
    Blob_Set_Index b_parent = b->parent_index;
    /*
    fprintf(stderr, "yuv_run_union: row %d a= (%hu %hu %hu) b= (%hu %hu %hu): ",
           a_row, a_parent, a->run_low, a->run_high,
           b_parent, b->run_low, b->run_high); // DEBUG
    */
    if (a_parent == 0 && b_parent == 0) {
        /* Both a and b are singletons.  I.e. neither has yet been added to
           a Blob_Set.  Make a new Blob_Set for a, then add b to it. */

        a_parent = make_set(p, a, a_row);
        /* If make_set runs out of memory it returns 0.  Hence, all the
           stats for blobs for which there is no memory will collect in
           p->blob_set[0], which is reserved for this purpose. */
        b->parent_index = a_parent;
        assert(a_parent < p->used_blob_set_count);
        stats_add_singleton(&p->blob_set[a_parent].stats, b_row, b->run_low,
                            b->run_high);
        //fprintf(stderr, "make_set(%d %d) %hu\n", 0, 0, a_parent);
    } else {
        // At most one of a_parent and b_parent is 0.
        unsigned short b_run_low = b->run_low;
        unsigned short b_run_high = b->run_high;
        if (a_parent == 0) {
            // Swap a and b, so that a_parent is not 0.
            Blob_Set_Index swap_tmp = a_parent;
            a_parent = b_parent;
            b_parent = swap_tmp;
            b_run_low = a->run_low;
            b_run_high = a->run_high;
            b_row = a_row;
            Yuv_Run* swap_tmp2 = a;
            a = b;
            b = swap_tmp2;
        }
        // Now we know a_parent is not 0.
        if (b_parent == 0) {
            a_parent = find_root(p, a_parent);
            // simplified link
            b->parent_index = a_parent;
            assert(a_parent < p->used_blob_set_count);
            stats_add_singleton(&p->blob_set[a_parent].stats, b_row, b_run_low,
                                b_run_high);
            //fprintf(stderr, "simple(%hu %hu) %hu\n", a_parent, b_parent, a_parent);
        } else {
            // Neither a_parent nor b_parent is 0.
            a_parent = find_root(p, a_parent);
            b_parent = find_root(p, b_parent);
            if (a_parent != b_parent) {
                b->parent_index = link(p, a_parent, b_parent);
                //fprintf(stderr, "link(%hu %hu) -> %hu\n", a_parent, b_parent, b->parent_index);
            } else {
                //fprintf(stderr, "equal(%hu %hu) -> %hu\n", a_parent, b_parent, b_parent);
            }
        }
    }
    a->parent_index = a_parent;

    /*
    fprintf(stderr, "root_list=");
    int ii;
    for (ii = 0; ii < p->used_root_list_count; ++ii) {
        fprintf(stderr, " %hu", p->root_list[ii]);
    }
    fprintf(stderr, "\n");
    */
}
/* Run through a single row of y values; for any pixel whose y value is above
   threshold and has u and v values within threshold add that pixel to a
   yuv_run entry. */
static unsigned int detect_yuv_runs_in_row(Blob_List* p,
                                           int cols,
                                           int rows,
                                           unsigned char y_row[],
                                           int uv_run_count,
                                           Uv_Run uv_run[],
                                           unsigned char y_low,
                                           Yuv_Run yuv_run[]) {
    int yuv_run_count = 0;
    int uvi = 0;
    if (uv_run_count < 1) return 0;
    int yi = uv_run[0].run_low;
    while (yi < cols) {
        /* Advance yi to search for start of a new yuv_run. */
        while (yi < uv_run[uvi].run_high && y_row[yi] < y_low) ++yi;
        if (yi < uv_run[uvi].run_high) {
            /* Some y value inside the uv_run is above threshold.
               Start new yuv_run. */
            yuv_run[yuv_run_count].run_low = yi;
            ++yi;
            /* Advance till you exit the uv_run, or the y value goes below
               threshold. */
            while (yi < uv_run[uvi].run_high && y_row[yi] >= y_low) ++yi;
            /* Stop the run. */
            yuv_run[yuv_run_count].run_high = yi;
            yuv_run[yuv_run_count].parent_index = 0;
            ++yuv_run_count;
        }
        if (yi >= uv_run[uvi].run_high) {
            /* We are done with this uv_run entry.  Move on to the next. */
            ++uvi;
            if (uvi >= uv_run_count) break;
            yi = uv_run[uvi].run_low;
        }
    }
    return yuv_run_count;
}


/* Any runs that overlap between run-list a and b are placed into the same
   disjoint set. */
static void yuv_run_row_union(Blob_List* p,
                              unsigned int a_row,
                              int a_count,
                              Yuv_Run a[],
                              unsigned int b_row,
                              int b_count,
                              Yuv_Run b[]) {
    int ai = 0;
    int bi = 0;
    int x = 0;
    while (ai < a_count && bi < b_count) {
        if (x < a[ai].run_low) {
            x = a[ai].run_low;
        }
        if (x < b[bi].run_low) {
            x = b[bi].run_low;
        }
        if (x < a[ai].run_high) {
            if (x < b[bi].run_high) {
                yuv_run_union(p, a_row, &a[ai], b_row, &b[bi]);
                if (a[ai].run_high < b[bi].run_high) {
                    ++ai;
                } else {
                    ++bi;
                }
            } else {
                ++bi;
            }
        } else {
            ++ai;
        }
    }
}

int detect_color_blobs(Blob_List* p,
                       unsigned char y_low,
                       unsigned char u_low,
                       unsigned char u_high,
                       unsigned char v_low,
                       unsigned char v_high,
                       bool highlight_detected_pixels,
                       int cols,
                       int rows,
                       unsigned char yuv[]) {
    /* Allocate memory for intermediate results. */

    int ii;

    /* The uv_run array contains runs in the current row of pixel sequences
       that meet thresholding in both U and V color values.  Y thresholding is
       done in a separate step because the U and V values are the same for
       contiguous even/odd row pairs.  Hence, we only need to calculate
       uv_run every other row. */

    const int max_uv_runs_in_row = cols / 4;
    Uv_Run* uv_run = (Uv_Run*)malloc(sizeof(Uv_Run) * max_uv_runs_in_row);
    int uv_run_count = 0; // number of used entries within uv_run

    /* The yuv_run0 array contains all Yuv_Runs from the previous row. */

    const int max_yuv_runs_in_row = cols / 2;
    Yuv_Run* yuv_run0 = (Yuv_Run*)malloc(sizeof(Yuv_Run) * max_yuv_runs_in_row);
    int yuv_run0_count = 0; // number of used entries within yuv_run0

    /* All Yuv_Runs from this row. */

    Yuv_Run* yuv_run1 = (Yuv_Run*)malloc(sizeof(Yuv_Run) * max_yuv_runs_in_row);

    /* Clear out the Blob_List. */

    blob_list_clear(p);

    for (ii = 0; ii < rows; ++ii) {
        /**** DEBUG ***
        if (ii == 432) {
            printf("row %d\n", ii);
        }
        */

        /* Process row ii of the image. */

        if ((ii & 0x01) == 0) {
            /* Even row.  Update uv_run. */

            int uv_offset = (ii / 2) * (cols / 2);
            int pixels = cols * rows;
            unsigned char* uv_row = &yuv[pixels + uv_offset];
            uv_run_count = create_uv_run_sequence(cols, rows, uv_row, u_low,
                                                  u_high, v_low, v_high,
                                                  uv_run);
        }

        /* Detect all YUV blob runs in the row. */

        unsigned char* y_row = &yuv[ii * cols];
        int yuv_run1_count = detect_yuv_runs_in_row(p, cols, rows, y_row,
                                                    uv_run_count, uv_run, y_low,
                                                    yuv_run1);

        /* (optional) Mark detected pixels in the image. */

        if (highlight_detected_pixels) {
            int jj;
            int kk = 0;
            for (jj = 0; jj < yuv_run1_count; ++jj) {
                /* Supress non-detected pixels. */
                while (kk < yuv_run1[jj].run_low) {
                    y_row[kk] /= 2;
                    ++kk;
                }
                /* Highlight detected pixels. */
                while (kk < yuv_run1[jj].run_high) {
                    int y = y_row[kk] * 2;
                    if (y > 255) y = 255;
                    y_row[kk] = y;
                    ++kk;
                }
            }
            /* Supress non-detected pixels to end of row. */
            while (kk < cols) {
                y_row[kk] /= 2;
                ++kk;
            }
        }

        /* If a run contains any 4-neighbors in common with a run from the
           previous row, add it to the Blob_List. */

        yuv_run_row_union(p, ii - 1, yuv_run0_count, yuv_run0,
                          ii, yuv_run1_count, yuv_run1);

        /* Set up yuv_run0 for next loop. */

        Yuv_Run* swap_tmp = yuv_run0;
        yuv_run0 = yuv_run1;
        yuv_run1 = swap_tmp;
        yuv_run0_count = yuv_run1_count;
    }
    free(yuv_run1);
    free(yuv_run0);
    free(uv_run);

    return 0;
}

static inline bool is_less(Blob_List* p, Blob_Set_Index a, Blob_Set_Index b) {
    int a_pix_count = get_blob_stats(p, a)->count;
    int b_pix_count = get_blob_stats(p, b)->count;
    return a_pix_count < b_pix_count;
}

/**
 * Here is the gap-sequence used by shell sort.  This gap sequence
 * is recommended in
 *   Marcin Ciura, Best Increments for the Average Case of Shellsort,
 *        13th International Symposium on Fundamentals of Computation Theory,
 *        Riga, Latvia, 22-24 August 2001;
 *        Lecture Notes in Computer Science 2001; Vol. 2138, pp. 106-117.
 */
static const int inc_schedule[] = { 1, 4, 10, 23, 57, 132, 301, 701 };
#define INC_SCHEDULE_SIZE ((int)(sizeof(inc_schedule)/sizeof(inc_schedule[0])))


void shell_sort_blobs_by_pixel_count(Blob_List* p) {

    int arr_size = p->used_root_list_count;
    Blob_Set_Index* arr = p->root_list;
    int k;
    for (k = INC_SCHEDULE_SIZE - 1; k >= 0; --k) {
        int i;
        int inc = inc_schedule[k];
        for (i = inc; i < arr_size; ++i) {
            int j = i;
            Blob_Set_Index temp_arr = arr[i];
            while (j >= inc && is_less(p, arr[j - inc], temp_arr)) {
                arr[j] = arr[j - inc];
                j -= inc;
            }
            arr[j] = temp_arr;
        }
    }
}


unsigned int copy_best_bounding_boxes(Blob_List* p,
                                      unsigned int min_pixels_per_blob,
                                      int bbox_element_count,
                                      unsigned short bbox_element[]) {
    int i;
    int k = 0;
    shell_sort_blobs_by_pixel_count(p);
    for (i = 0; i < p->used_root_list_count; ++i) {
        Blob_Set_Index ii = p->root_list[i];
        Blob_Stats* s = &p->blob_set[ii].stats;
        printf("pix_count= %u\n", s->count);
        if (s->count >= min_pixels_per_blob) {
            bbox_element[k*4+0] = s->min_x;
            bbox_element[k*4+1] = s->min_y;
            bbox_element[k*4+2] = s->max_x;
            bbox_element[k*4+3] = s->max_y;
            ++k;
            if (k >= bbox_element_count / 4) break;
        }
    }
    return k * 4;
}


void draw_bounding_boxes(const Blob_List* p,
                         int min_pixels_per_blob,
                         const unsigned char bbox_color_yuv[3],
                         int cols,
                         int rows,
                         unsigned char yuv[]) {
    int i;
#define DSC_DEBUG
#ifdef DSC_DEBUG
    fprintf(stderr,"============================================\n");
#endif
    /* DSC DEBUG */
    int how_many = p->used_root_list_count;
    if (how_many > 2) how_many = 2;
    /* DSC DEBUG END */
    for (i = 0; i < how_many; ++i) {
        Blob_Set_Index ii = p->root_list[i];
        Blob_Stats* s = &p->blob_set[ii].stats;
        if (s->count >= min_pixels_per_blob) {
#ifdef DSC_DEBUG
            fprintf(stderr, "bbox: (%d %d) (%d %d)\n", s->min_x, s->min_y, s->max_x, s->max_y);
#endif
            const int pixels = cols * rows;

            // Draw top and bottom lines.

            unsigned short x;
            unsigned char* y_top_row = &yuv[s->min_y * cols];
            unsigned char* y_bot_row = &yuv[s->max_y * cols];
            int u_top_offset = (s->min_y / 2) * (cols / 2);
            unsigned char* u_top_row = &yuv[pixels + u_top_offset];
            unsigned char* v_top_row = u_top_row + (pixels / 4);
            int u_bot_offset = (s->max_y / 2) * (cols / 2);
            unsigned char* u_bot_row = &yuv[pixels + u_bot_offset];
            unsigned char* v_bot_row = u_bot_row + (pixels / 4);
            for (x = s->min_x; x <= s->max_x; ++x) {
                y_top_row[x] = bbox_color_yuv[0];
                y_bot_row[x] = bbox_color_yuv[0];
            }
            for (x = s->min_x / 2; x <= s->max_x / 2; ++x) {
                u_top_row[x] = bbox_color_yuv[1];
                u_bot_row[x] = bbox_color_yuv[1];
                v_top_row[x] = bbox_color_yuv[2];
                v_bot_row[x] = bbox_color_yuv[2];
            }

            // Draw left and right lines.

            unsigned short y;
            unsigned char* y_left_col = &yuv[s->min_x];
            unsigned char* y_right_col = &yuv[s->max_x];
            int u_left_offset = s->min_x / 2;
            unsigned char* u_left_col = &yuv[pixels + u_left_offset];
            unsigned char* v_left_col = u_left_col + (pixels / 4);
            int u_right_offset = s->max_x / 2;
            unsigned char* u_right_col = &yuv[pixels + u_right_offset];
            unsigned char* v_right_col = u_right_col + (pixels / 4);
            for (y = s->min_y; y <= s->max_y; ++y) {
                y_left_col[y * cols] = bbox_color_yuv[0];
                y_right_col[y * cols] = bbox_color_yuv[0];
            }
            for (y = s->min_y / 2; y <= s->max_y / 2; ++y) {
                u_left_col[y * (cols / 2)] = bbox_color_yuv[1];
                u_right_col[y * (cols / 2)] = bbox_color_yuv[1];
                v_left_col[y * (cols / 2)] = bbox_color_yuv[2];
                v_right_col[y * (cols / 2)] = bbox_color_yuv[2];
            }
        }
    }
}

Blob_List blob_list_init(Blob_Set_Index max_runs, Blob_Set_Index max_blobs) {
    Blob_List bl;
    bl.blob_set = (Blob_Set_Entry*)malloc(
                                     sizeof(Blob_Set_Entry) * (max_runs + 1));
    bl.max_blob_set_count = max_runs + 1;
    bl.root_list = (Blob_Set_Index*)malloc(sizeof(Blob_Set_Index) * max_blobs);
    bl.max_root_list_count = max_blobs;
    if (bl.blob_set == NULL || bl.root_list == NULL) {
        free(bl.blob_set);
        free(bl.root_list);
        bl.blob_set = NULL;
        bl.max_blob_set_count = 0;
        bl.root_list = NULL;
        bl.max_root_list_count = 0;
    }
    blob_list_clear(&bl);
    return bl;
}

void blob_list_deinit(Blob_List* p) {
    free(p->blob_set);
    free(p->root_list);
}
void blob_list_clear(Blob_List* p) {
    p->used_blob_set_count = 1;  // index 0 is reserved
    p->used_root_list_count = 0;

    /* Initialize blob_set[0]. */

    p->blob_set[0].parent_index = 0;  // self pointer
    /* Root_list_index is only used to allow us to delete this entry from the
       root_list.  Since this entry is not on the root_list, it should never
       be used.  Give it a value anyway. */
    p->blob_set[0].root_list_index = 0;
    p->blob_set[0].rank = 0;
    p->blob_set[0].stats.max_x = 0;
    p->blob_set[0].stats.min_x = USHRT_MAX;
    p->blob_set[0].stats.max_y = 0;
    p->blob_set[0].stats.min_y = USHRT_MAX;
    p->blob_set[0].stats.sum_x = 0;
    p->blob_set[0].stats.sum_y = 0;
}
