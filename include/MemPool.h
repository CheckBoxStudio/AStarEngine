/*******************************************************************************
 * MemPool.h
 * For fast memory management such as allocation and free.
 * Based on 'fsa.h' from 'https://github.com/weiquanmao/astar-algorithm-cpp'.
 *
 * History
 * ----------------
 * Created. 04 Oct. 2018. WeiQM<weiquanmao@hotmail.com>
 *
 * <Encodeing in UTF-8>
 * =============================================================================
 * Permission is given by the author to freely redistribute and
 * include this code in any program as long as this credit is
 * given where due.
 *
 *  COVERED CODE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS,
 *  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 *  INCLUDING, WITHOUT LIMITATION, WARRANTIES THAT THE COVERED CODE
 *  IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE
 *  OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND
 *  PERFORMANCE OF THE COVERED CODE IS WITH YOU. SHOULD ANY COVERED
 *  CODE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL
 *  DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY
 *  NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF
 *  WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE
 *  OF ANY COVERED CODE IS AUTHORIZED HEREUNDER EXCEPT UNDER
 *  THIS DISCLAIMER.
 *
 *  Use at your own risk!
 *******************************************************************************/

#ifndef MEMPOOL_H
#define MEMPOOL_H

#include <stdio.h>
#include <assert.h>

#define _Check_Valid_

template <typename UserDataType> class MemPool
{
public:
    struct MemPoolElement
    {
        UserDataType userData;
        MemPoolElement *pPrev;
        MemPoolElement *pNext;
    };

public:
    MemPool(unsigned int MemPoolSize)
        : m_MemPoolSize(0)
        , m_pMemory(NULL)
        , m_pMemVacant(NULL)
        , m_pMemUsing(NULL)
    {
        AllocMem(MemPoolSize);
    }

    ~MemPool()
    {
        FreeMem();
    }

    int AllocMem(const unsigned MemPoolSize)
    {
        FreeMem();
        if (MemPoolSize)
        {
            // allocate memory buffer
            char *pBuff = new(std::nothrow) char[MemPoolSize*sizeof(MemPoolElement)];
            assert(pBuff != NULL);
            if (pBuff == NULL) {
                return -1;
            }

            // clear the memory
            memset(pBuff, 0, MemPoolSize*sizeof(MemPoolElement));

            // set double linked
            MemPoolElement *pMem = (MemPoolElement*) pBuff;
            MemPoolElement *pElement = pMem;
            for(unsigned int i=0; i<MemPoolSize; ++i) {
                pElement->pPrev = pElement-1;
                pElement->pNext = pElement+1;
                pElement++;
            }
            pMem->pPrev = NULL;
            (pElement-1)->pNext = NULL;

            // use the new allocated memory
            m_pMemory = pMem;
            m_pMemVacant = pMem;
            m_MemPoolSize = MemPoolSize;
        }
        return m_MemPoolSize;
    }

    void FreeMem()
    {
        if (m_pMemory) {
            delete[] (char*) m_pMemory;
            m_pMemory = NULL;
            m_pMemVacant = NULL;
            m_pMemUsing = NULL;
            m_MemPoolSize = 0;
        }
    }

    unsigned int MemSize() const
    {
        return m_MemPoolSize;
    }

    // allocate a new UserDataType and return a pointer to it
    UserDataType* Alloc()
    {
        MemPoolElement *pNewMPElement = NULL;
        if(m_pMemVacant)
        {
            pNewMPElement = m_pMemVacant;

            // remove one element from the vacant list
            m_pMemVacant = m_pMemVacant->pNext;
            if (m_pMemVacant) {
                m_pMemVacant->pPrev = NULL;
            }

            // add it to the using list
            pNewMPElement->pPrev = NULL;
            pNewMPElement->pNext = m_pMemUsing;
            if(m_pMemUsing) {
                m_pMemUsing->pPrev = pNewMPElement;
            }
            m_pMemUsing = pNewMPElement;
        }
        return reinterpret_cast<UserDataType*>(pNewMPElement);
    }

    // free the given user data
    void Free(UserDataType* userDataElement)
    {
        if (userDataElement)
        {
            MemPoolElement *pMPElement = reinterpret_cast<MemPoolElement*>(userDataElement);
#ifdef _Check_Valid_
            assert(pMPElement);
            assert(pMPElement >= m_pMemory);
            assert(pMPElement < m_pMemory+m_MemPoolSize);
#endif
            // remove it from the using list
            if (pMPElement->pPrev) {
                pMPElement->pPrev->pNext = pMPElement->pNext;
            }
            else {
                // assert(pMPElement == m_pMemUsing);
                m_pMemUsing = pMPElement->pNext;
            }
            if(pMPElement->pNext) {
                pMPElement->pNext->pPrev = pMPElement->pPrev;
            }

            // add it to the vacant list
            pMPElement->pPrev = NULL;
            pMPElement->pNext = m_pMemVacant;
            if(m_pMemVacant) {
                m_pMemVacant->pPrev = pMPElement;
            }
            m_pMemVacant = pMPElement;
        }
    }

    // for debugging this displays both lists (using the prev/next list pointers)
    void Debug()
    {
        printf( "[--vacant list--] >> \n" );
        int n = 0;
        MemPoolElement *p = m_pMemVacant;
        if (!p) {
            printf(" (null) \n");
        }
        while(p) {
            printf( "%08x:%08x:%08x  ", p->pPrev, p, p->pNext );
            p = p->pNext;
            n++;
            if (n == 4) {
                printf("\n");
                n = 0;
            }
        }
        if (n != 0) {
            printf("\n");
        }
        printf("\n");

        printf( "[--using list--] >> \n" );
        p = m_pMemUsing;
        n = 0;
        if (!p) {
            printf(" (null) \n");
        }
        while(p) {
            printf( "%08x:%08x:%08x ", p->pPrev, p, p->pNext );
            p = p->pNext;
            n++;
            if (n == 4) {
                printf("\n");
                n = 0;
            }
        }
        if (n != 0) {
            printf("\n");
        }
        printf("\n");
    }

    // iterators
    UserDataType* GetFirst()
    {
        return reinterpret_cast<UserDataType*>(m_pMemUsing);
    }

    UserDataType* GetNext(UserDataType* userDataElement)
    {
        MemPoolElement *pNext = NULL;
        if (userDataElement)
        {
            MemPoolElement *pMPElement = reinterpret_cast<MemPoolElement*>(userDataElement);
#ifdef _Check_Valid_
            assert(pMPElement);
            assert(pMPElement >= m_pMemory);
            assert(pMPElement < m_pMemory+m_MemPoolSize);
#endif
            pNext = pMPElement->pNext;
        }
        return reinterpret_cast<UserDataType*>(pNext);
    }

private:
    unsigned int m_MemPoolSize;
    MemPoolElement *m_pMemory;
    MemPoolElement *m_pMemVacant;
    MemPoolElement *m_pMemUsing;
};

#endif // !MEMPOOL_H