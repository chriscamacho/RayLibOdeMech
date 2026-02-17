
#include <stdio.h>
#include <stdbool.h>


#include <stdlib.h>
#include "clist.h"

/** @brief
 *   creates a list
 *
 *   @details
 *   Allocates space for the list structure clearing the
 *   head and tail pointer ready for use.
 *
 *   @return provides a pointer to the newly created list
 */
clist_t*clistCreateList()
{
    clist_t* new = malloc(sizeof(clist_t));

    if (!new) {
        printf("Couldn't allocate memory for a list\n");
        // NB this mini library was previously released with the onus on
        // the end user to check return values, I ended up deleting the 
        // repo because I was sick of people not reading documentation
        // and posting issues, so instead you get an exit...
        exit(-1);
    }
    new->head = NULL;
    new->tail = NULL;
    return(new);
}

/** @brief
 *   adds a node to a list
 *
 *   @details
 *   allocates space for a new node and places the node at
 *   the end of the list
 *
 *   @param [in] list the list to add the node to
 *   @param [in] data the data that this node points to
 *
 *   @return the newly allocated node
 */
cnode_t*clistAddNode(clist_t* list, void* data)
{
    cnode_t* new = malloc(sizeof(cnode_t));

    if (!new) {
        printf("Couldn't allocate memory for a list\n");
        exit(-1);
    }
    new->prev = list->tail;
    new->next = NULL;
    if (new->prev != NULL) {
        new->prev->next = new;
    }
    if (list->head == NULL) {
        list->head = new;
    }
    list->tail = new;
    new->data  = data;
    return(new);
}

/** @brief
 *   inserts a node at any point in the list
 *
 *   @details
 *   The newly created node takes the position of the inserted
 *   node, with the insert node and all subsequent nodes coming after
 *   it in the list order
 *
 *   @param [in] list the list you want to insert the node into
 *   @param [in] node the node in the list at the position
 *   you want to insert the new node at
 *   @param [in] data the data new node points to
 *
 *   @return the newly created node is returned
 */
cnode_t*clistInsertNode(clist_t* list, cnode_t* node, void* data)
{
    cnode_t* new = malloc(sizeof(cnode_t));

    if (!new) {
        printf("Couldn't allocate memory for a node\n");
        exit(-1);
    }
    new->next = node;
    new->prev = node->prev;
    if (node->prev != NULL) {
        node->prev->next = new;
    }
    node->prev = new;
    if (list->head == node) {
        list->head = new;
    }
    new->data = data;
    return(new);
}

/** @brief
 *   search for a node in the list
 *
 *   @details
 *   brute force search for a node pointing to specific data
 *   generally you should be iterating lists and not dealing
 *   with specific nodes individually but this could come
 *   in handy
 *
 *   @param [in] list the list you want to find the node in
 *   @param [in] ptr finds the node that points to this data
 *
 *   @return checks each node in turn looking for a node that
 *   points to ptr returns NULL if it doesn't find a match
 */
cnode_t*clistFindNode(clist_t* list, void* ptr)
{
    cnode_t* node = list->head;

    while (node != NULL) {
        if (node->data == ptr) {
            return(node);
        }
        node = node->next;
    }
    node = NULL;
    return(node);
}

/** @brief
 *   delete a node
 *
 *   @details this unlinks the node from the list and then frees its
 *   memory (the user is responsible for the data the nodes data pointer points to)
 *   the pointer to the node (pnode) is then NULLed to aid bug and resource
 *   checking
 *
 *
 *   @param [in] list the list containing the node you wish to delete
 *   @param [in, out] pnode a pointer to the node pointer you want to delete, 
 * 			the node pointer is returned NULLed
 */
void clistDeleteNode(clist_t* list, cnode_t** pnode)
{
    cnode_t* node = *pnode;

    if (node->prev != NULL) {
        node->prev->next = node->next;
    }
    if (node->next != NULL) {
        node->next->prev = node->prev;
    }
    if (list->head == node) {
        list->head = node->next;
    }
    if (list->tail == node) {
        list->tail = node->prev;
    }
    free(*pnode);
    *pnode = 0;
}

/** @brief
 *   delete a node (containing a pointer)
 *
 *   @details this unlinks the node from the list and then frees its
 *   memory (the user is responsible for the data the nodes data pointer points to)
 *   the pointer to the node (pnode) is then NULLed to aid bug and resource
 *   checking the node is found by searching for a node with a matching
 *   data pointer
 *
 *   @param [in] list the list containing the node you wish to delete
 *   @param [in] ptr a pointer to data, for selecting the node
 */
void clistDeleteNodeFromData(clist_t* list, void* ptr)
{
    cnode_t* n;

    n = clistFindNode(list, ptr);
    if (n == NULL) {
        return;
    }
    clistDeleteNode(list, &n);
}

/** @brief
 *   empty a list
 *
 *   @details
 *   this will iterate the list freeing all the nodes in it.
 *   The list is still valid for use afterwards
 *
 *   @param [in] list the list you want emptying
 */
void clistEmptyList(clist_t* list)
{
    cnode_t* node;

    while (list->head != NULL) {
        node = list->head; // protect head from nulling by deleteNode
        clistDeleteNode(list, &node);
    }
}

/** @brief free a list and all its resources
 *
 * @details
 * this empties and frees all resources used by the list.
 * Once freed it cannot be used...<br>
 * This does not free what a node is pointing to just
 * the nodes data structure.
 *
 * @param [in,out] list The address of the list pointer. It will be freed 
 * and then set to NULL to aid bug & resource checking.
 */
void clistFreeList(clist_t** list)
{
    clistEmptyList(*list);
    free(*list);
    *list = NULL;
}

/** @brief
 *   iterate a list
 *
 *   @details
 *   This moves forward through a list calling a callback function
 *   for each node in the list. If a callback is not apropriate, because for
 *   example you need to access local variables then you should iterate the list
 *   in this manner
 *
 *   @code
 *       cnode_t *node=list->head;
 *       while (node!=NULL) {
 *           // your code here
 *           node=node->next;
 *       }
 *   @endcode
 *
 *   @param [in] list the list to iterate
 *   @param [in] nodeFunction the callback should return nothing and take
 *   only a node pointer as its parameter
 */
void clistIterateForward(clist_t* list, void (*nodeFunction)(cnode_t* node))
{
    cnode_t* node = list->head;

    while (node != NULL) {
        nodeFunction(node);
        node = node->next;
    }
}

/** @brief
 *        Iterate backwards through a list
 *
 *   @details
 *   identical to clistIterateForwards execept in direction
 *
 *   @param [in] list the list to iterate
 *   @param [in] nodeFunction the callback should return nothing and take
 *   only a node pointer as its parameter
 */
void clistIterateBackward(clist_t* list, void (*nodeFunction)(cnode_t* node))
{
    cnode_t* node = list->tail;

    while (node != NULL) {
        nodeFunction(node);
        node = node->prev;
    }
}

/** @brief
 * sort a list
 *
 * @details
 * does a simple bubble sort on a list depending on node compare callback
 *
 * @param [in] list         the list to iterate
 * @param [in] cmpFunction  the callback which should return 1 if n1 > n2
 */
void clistSort(clist_t* list, int(*cmpFunction)(cnode_t* n1, cnode_t* n2))
{
    bool      hasSwapped = true;
    cnode_t* node1, * node2;

    while (hasSwapped == true) {
        hasSwapped = false;
        node1      = list->head;
        while (node1 != NULL) {
            node2 = node1->next;
            if (node2 != NULL) {
                if (cmpFunction(node1, node2) > 0) {
                    void* v;
                    v = node1->data;
                    node1->data = node2->data;
                    node2->data = v;
                    hasSwapped  = true;
                }
            }
            node1 = node1->next;
        }
    }
}

/** @brief number of items in list
 *
 *   @details This iterrates the list counting all the nodes
 *
 *   @param [in] list the list you wish to total
 *   @result [out] the number of nodes in the list
 */
int clistTotal(clist_t* list)
{
    int      c    = 0;
    cnode_t* node = list->head;

    while (node != NULL) {
        c++;
        node = node->next;
    }
    return(c);
}

/** @brief is list empty?
 *
 *   @details test to see if the list is empty
 *
 *   @param [in] list
 *   @return 1 if empty, 0 if at least 1 node
 */
int clistIsEmpty(clist_t* list)
{
    return(!list->head);
}
