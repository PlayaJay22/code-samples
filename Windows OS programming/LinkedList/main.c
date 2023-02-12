#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* A program to implement a linked list with various operations */

typedef struct node {
    int val;
    struct node* next;
} node_t;

void insertBeginning(node_t** head, int val);
void insertEnd(node_t** head, int val);
void insertAtPosition(node_t** head, int val, int pos);
void deletePosition(node_t** head, int pos);
void search(node_t* head, int val);
void updateValue(node_t* head, int val, int pos);
void displayList(node_t* head);
void freeList(node_t* head);

int main()
{
    node_t* head = NULL;
    int ch = '\0';
    int val = 0;

    printf("\n---------------------------------\n");
    printf("\nOperations on a linked list\n");
    printf("\n---------------------------------\n");

    while (true)
    {

        printf("\n1.Insert node at beginning");
        printf("\n2.Insert node at end");
        printf("\n3.Insert node at a specific position");
        printf("\n4.Delete Node from any Position");
        printf("\n5.Update Node Value");
        printf("\n6.Search Element in the linked list");
        printf("\n7.Display List");
        printf("\n8.Exit\n");
        printf("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        printf("\nEnter your choice: ");
        scanf("%d", &ch);

        switch(ch)
        {
            case 1:
                printf("Enter value to insert:");
                scanf("%d", &val);
                insertBeginning(&head,val);
                break;
            case 7:
                displayList(head);
                break;
            case 8:
                printf("\n...Exiting...\n");
                freeList(head);
                return 0;
            default:
                printf("\n...Invalid Choice...\n");
                break;

        }
    }




    return 0;
}

void insertBeginning(node_t** head, int val)
{
    node_t* newNode = malloc(sizeof(node_t));  //create a new node in memory
    newNode->val = val; //assign data value to new node

    if(*head != NULL) {  //this means the head already contains data
        newNode->next = *head; //point new node to head (the data at the start of the list)
        *head = newNode; //move the head pointer to the start of the list again
    }

    else { //head is empty
        *head = newNode; //assign head to the first node with the new data
        (*head)->next = NULL; //we are at the end of the list
    }
}

void insertEnd(node_t** head, int val)
{
    node_t* newNode = malloc(sizeof(node_t)); //create a new node
    newNode->val = val; //populate new node
    newNode->next = NULL; //this node will be put at the end

    if(*head != NULL) //this means the list has at least 1 node already
    {
        node_t* temp = *head; //used to traverse list

        while(temp->next != NULL) //this means, we've not reached the end of the list. Keep going...
            temp = temp->next;
        //at this point, temp points to the last node

        temp->next = newNode;// point the last node to the newNode, adding the newNode to the end.
    } else  //list is currently empty
        *head = newNode; //head becomes the newNode;

}

/*
We have to cater for all the following situations
* The list is empty: insert at head
* The list has 1 node: insert at head
* The list has 2+ nodes: insert normally
* The desired pos is greater than the node count: insert at end
*/
void insertAtPosition(node_t** head, int val, int pos)
{
    node_t* newNode = malloc(sizeof(node_t));  //create new node in memory
    newNode->val = val; //populate node

    if(*head != NULL) //only continue if there is at least 1 node
    {
        if((*head)->next != NULL) //this means the list has at least 2 nodes already
        {
            if(pos == 1) //cater for special case where user wants to insert at pos 1
            {
                newNode->next = *head;
                *head = newNode;
            } else {
                //now for the regular case where pos > 1
                int currentPos = 2;
                node_t* prevNode = *head; //initialize to head
                node_t* nextNode = (*head)->next;  //initialize to node after head
                while(currentPos < pos) //keep looping until we are at the desired node position
                {
                    prevNode = nextNode; //move forward one node
                    nextNode = nextNode->next; //move forward one node
                    currentPos++;

                    if(nextNode == NULL)  //if the desired position is larger than the node count, break prematurely
                        break;
                }
                newNode->next = nextNode;
                prevNode->next = newNode;
            }
        }
    } else //list is empty
    {
        *head = newNode;
        newNode->next = NULL;
    }
}

void deletePosition(node_t** head, int pos)
{
    printf("attempting to delete position %d...\n",pos);

    if(*head == NULL) //list empty
    {
        printf("list is already empty\n");
    } else
    {
        int count = 2;
        node_t* prevNode = *head;
        node_t* temp = (*head)->next;

        if(pos == 1)
        {
            if((*head)->next == NULL) //there is only 1 node
            {
                free(*head);
                *head = NULL;
            } else  //there is more than one node
            {
                temp = *head;
                *head = (*head)->next;
                free(temp);
            }
        } else { //pos > 1
            while((count < pos) && (temp != NULL)) //stop if we reach the desired position, or end of the list
            {
                prevNode = temp;
                temp = temp->next;
                count++;
            }
            if(temp != NULL) //only free temp if it is a valid node
            {
                prevNode->next = temp->next;  //jump ahead
                free(temp); //kill node at pos
            } else {
                printf("out of bounds\n");
            }
        }

    }
}

void search(node_t* head, int val)
{
    int count = 1;

    if(head == NULL)
        printf("list is empty\n");
    else {
        while((head->val != val) && (head != NULL))
        {
            head = head->next;
            count++;
        }

        if(head != NULL) //we found it!!
            printf("value %d found at position %d\n",val,count);
        else
            printf("we did not find value %d\n",val);
    }
}

void updateValue(node_t* head, int newVal, int pos)
{
    int count = 1;

    if(head == NULL)
        printf("list is empty\n");
    else {
        while((count < pos) && (head != NULL))
        {
            head = head->next;
            count++;
        }
        if(head != NULL) //we found it!!
        {
            printf("Updating value %d at position %d...\n", head->val, count);
            head->val = newVal;
        }
        else
            printf("Out of bounds\n");

    }
}

void displayList(node_t* head)  //send in a copy of the head pointer
{
    printf("The list contains the following:\n");
    while(head != NULL)
    {
        printf("%d --> ",head->val); //print value
        head = head->next; //move head to next node
    }
    printf("NULL\n");
}

void freeList(node_t* head)
{
    node_t* temp;

    while(head != NULL)
    {
        temp = head;  //store current head in temp
        head = head->next;  //move head onwards
        free(temp); //free the original head.
    }
}
