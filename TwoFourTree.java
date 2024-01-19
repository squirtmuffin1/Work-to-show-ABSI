package termproject;
//"C:\Users\natha\StudioProjects\cs2210termprojectfa23-squirtmuffin1\src\termproject"

/**
 * Title:        Term Project 2-4 Trees
 * Description:
 * Copyright:    Copyright (c) 2001
 * Company:
 * @author David Moore and Nathaniel Cavallaro
 * @version 1.0
 */
public class TwoFourTree
        implements Dictionary {

    private Comparator treeComp;
    private int size = 0; //I assume this is based on the number of nodes, not items.
    private TFNode treeRoot = null;

    public TwoFourTree(Comparator comp) {
        treeComp = comp;
    }

    private TFNode root() {
        return treeRoot;
    }

    private void setRoot(TFNode root) {
        treeRoot = root;
    }

    public int size() {
        return size;
    }

    public boolean isEmpty() {
        return (size == 0);
    }

    //Helper function
    public int whatChildIsThis(TFNode node) {
        TFNode parent = node.getParent();
        int i = 0;
        for (i = 0; i < parent.getNumItems() + 1; i++) {
            if (parent.getChild(i) == node) {
                break;
            }
        }
        return i;
    }

    /**
     * Searches dictionary to determine if key is present
     * @param key to be searched for
     * @return object corresponding to key; null if not found
     */

    //Helper function
    private int findFirstGreaterThanOrEqual(TFNode node, Object key){
        int i;
        for(i = 0; i < node.getNumItems(); i++){
            if(treeComp.isGreaterThanOrEqualTo(node.getItem(i).key(), key)){
                break;
            }
        }
        return i;
    }
    public TFNode find(Object key){
        TFNode node;
        if (size == 0) {
            setRoot(new TFNode());
            size++;
        }
        node = treeRoot;

        int index = findFirstGreaterThanOrEqual(node, key);

        while(node.getNumItems() >= index && node.getChild(index) != null) {
            if(node.getNumItems() > index &&
                    treeComp.isEqual(node.getItem(index).key(), key)){
                break;
            }
            node = node.getChild(index);
            index = findFirstGreaterThanOrEqual(node, key);
        }

        return node;
    }



    public Object findElement(Object key){ // see if needs to return element not just the item
        //return find(key).getItem(findFirstGreaterThanOrEqual(find(key), key));

        //I made some changes for readability and I also tweaked the logic a little.
        //Will this work? Is there a way we can test it without insert() and remove()?
        TFNode node = find(key);
        Integer itemIndex = findFirstGreaterThanOrEqual(node, key);
        if (node.getNumItems() >= itemIndex + 1) {
            return node.getItem(itemIndex);
        }
        else { return null; }
    }

    /**
     * Inserts provided element into the Dictionary
     * @param key of object to be inserted
     * @param element to be inserted
     */
    public void insertElement(Object key, Object element) {
        Item foundElement = (Item) findElement(key);
        TFNode node = find(key);
        int index = findFirstGreaterThanOrEqual(node,key);

        //Case for inserting a new element (a non-duplicate)
        if (foundElement == null && node.getChild(0) == null) {
            foundElement = new Item(key, element);
            node.insertItem(index, foundElement);
        }
        //Case for inserting a duplicate element when the original element
        //is NOT in a leaf.
        else if (node.getChild(index + 1) != null) {
            foundElement = new Item(key, element);
            node = node.getChild(index + 1);
            while(node.getChild(0) != null){
                node = node.getChild(0);
            }
            node.insertItem(0,foundElement);
        }
        //Case for inserting a duplicate element when the original element
        //IS in a leaf.
        else {
            foundElement = new Item(key, element);
            node.insertItem(index,foundElement);
        }

        if(node.getNumItems() > 3){
            performOverflow(node);
        }
    }

    /**
     * Searches dictionary to determine if key is present, then
     * removes and returns corresponding object
     * @param key of data to be removed
     * @return object corresponding to key
     * @exception ElementNotFoundException if the key is not in dictionary
     */
    public Object removeElement(Object key) throws ElementNotFoundException {
        Item itemToBeRemoved = (Item) findElement(key);
        TFNode node = find(key);
        int index = findFirstGreaterThanOrEqual(node,key);

        if (itemToBeRemoved == null) {
            throw new ElementNotFoundException("Error: You tried to remove" +
                    " an element that is not in the tree.");
        }

        if (node.getChild(0) == null) { //Item is in a leaf node
            node.removeItem(index);
            checkForUnderflow(node);
        }
        else {
            TFNode inOrderSuccessorNode = node.getChild(index + 1);
            while(!treeComp.isEqual(node.getChild(0), null)){
                node = node.getChild(0);
            }
            Item inOrderSuccessor = node.getItem(0);

            node.replaceItem(index,inOrderSuccessor);
            checkForUnderflow(inOrderSuccessorNode);
        }

        return itemToBeRemoved.element();
    }

    private void checkForUnderflow(TFNode node) {
        if (node.getItem(0) != null) { return; }

        int nodeIndex = whatChildIsThis(node);
        TFNode parent = node.getParent();
        TFNode leftSib;
        TFNode rightSib;

        if (nodeIndex == 0) { leftSib = null; }
        else { leftSib = parent.getChild(nodeIndex - 1); }

        if (nodeIndex == 3) { rightSib = null; }
        else { rightSib = parent.getChild(nodeIndex + 1); }

        if (leftSib != null && leftSib.getChild(1) != null) {
            leftTransfer(node);
        }
        else if (rightSib != null && rightSib.getChild(1) != null) {
            rightTransfer(node);
        }
        else if (leftSib != null) {
            leftFusion(node);
            checkForUnderflow(node.getParent());
        }
        else {
            rightFusion(node);
            checkForUnderflow(node.getParent());
        }

    }

    private void leftTransfer(TFNode node) {
        int nodeIndex  = whatChildIsThis(node);
        TFNode parent  = node.getParent();
        TFNode leftSib = parent.getChild(nodeIndex - 1);
        int numSibItems = leftSib.getNumItems();

        //The left item doesn't have to move.
        Item midItem   = leftSib.deleteItem(numSibItems - 1);
        Item rightItem = parent.deleteItem(nodeIndex - 1); //Maybe just nodeIndex?

        node.insertItem(0, rightItem);
        node.setChild(0,leftSib.getChild(numSibItems + 1));
        parent.replaceItem(nodeIndex - 1, midItem);
    }

    private void rightTransfer(TFNode node) {
        int nodeIndex  = whatChildIsThis(node);
        TFNode parent  = node.getParent();
        TFNode rightSib = parent.getChild(nodeIndex + 1);
        int numSibItems = rightSib.getNumItems();

        //The right item doesn't have to move.
        Item leftItem  = parent.deleteItem(nodeIndex);
        TFNode child = rightSib.getChild(0);
        Item midItem = rightSib.removeItem(0);

        node.insertItem(0, leftItem);
        node.setChild(0,child);
        parent.replaceItem(nodeIndex, midItem);
    }

    private void leftFusion(TFNode node){
        TFNode parent = node.getParent();
        TFNode leftSib = parent.getChild(whatChildIsThis(node) - 1);
        Item lItem = parent.removeItem(whatChildIsThis(node) - 1);
        leftSib.addItem(leftSib.getNumItems(), lItem);
        if(leftSib.getChild(0) != null){
            leftSib.setChild(leftSib.getNumItems(),node.getChild(0));
            node.getChild(0).setParent(leftSib);
        }
        parent.setChild(whatChildIsThis(node) - 1, leftSib);
    }

    private void rightFusion(TFNode node){
        TFNode parent = node.getParent();
        TFNode rightSib = parent.getChild(whatChildIsThis(node) + 1);
        Item rItem = parent.removeItem(whatChildIsThis(node));
        rightSib.insertItem(rightSib.getNumItems(), rItem);
        if(rightSib.getChild(0) != null){
            rightSib.setChild(0,node.getChild(0));
            node.getChild(0).setParent(rightSib);
        }
    }

    private void performOverflow(TFNode bigNode){
        //insert item into parent
        size++;
        Item item = bigNode.getItem(2);
        Object key = item.key();
        TFNode parent = bigNode.getParent();
        if (parent != null) {
            int index = findFirstGreaterThanOrEqual(parent, key);
            parent.insertItem(index,item);
        }
        else { //Create a new root node
            TFNode newRoot = new TFNode();
            newRoot.addItem(0, item);
            newRoot.setChild(0, bigNode);
            bigNode.setParent(newRoot);
            parent = newRoot;
            setRoot(parent);
            size++;
        }

        //fix up children
        TFNode leftNode = new TFNode();
        TFNode rightNode = new TFNode();
        leftNode.insertItem(0,bigNode.getItem(0));
        leftNode.insertItem(1,bigNode.getItem(1));
        rightNode.insertItem(0, bigNode.getItem(3));
        if(bigNode.getChild(0) != null){
            leftNode.setChild(0, bigNode.getChild(0));
            leftNode.setChild(1, bigNode.getChild(1));
            leftNode.setChild(2, bigNode.getChild(2));
            rightNode.setChild(0, bigNode.getChild(3));
            rightNode.setChild(1, bigNode.getChild(4));
            bigNode.getChild(0).setParent(leftNode);
            bigNode.getChild(1).setParent(leftNode);
            bigNode.getChild(2).setParent(leftNode);
            bigNode.getChild(3).setParent(rightNode);
            bigNode.getChild(4).setParent(rightNode);
        }
        leftNode.setParent(parent);
        rightNode.setParent(parent);

        //fix pointers to parent
        int index = whatChildIsThis(bigNode);
        parent.setChild(index, leftNode);
        parent.setChild(index + 1, rightNode);
        //check for overflow in the parent
        if(parent.getNumItems() > 3){ performOverflow(parent); }
    }

    public static void main(String[] args) {
        Comparator myComp = new IntegerComparator();
        TwoFourTree myTree = new TwoFourTree(myComp);

        Integer myInt1 = new Integer(47);
        myTree.insertElement(myInt1, myInt1);
        Integer myInt2 = new Integer(83);
        myTree.insertElement(myInt2, myInt2);
        Integer myInt3 = new Integer(22);
        myTree.insertElement(myInt3, myInt3);

        Integer myInt4 = new Integer(16);
        myTree.insertElement(myInt4, myInt4);

        //Our tree works at this point
        Integer myInt5 = new Integer(49);
        myTree.insertElement(myInt5, myInt5);

        Integer myInt6 = new Integer(100);
        myTree.insertElement(myInt6, myInt6);

        Integer myInt7 = new Integer(38);
        myTree.insertElement(myInt7, myInt7);

        Integer myInt8 = new Integer(3);  //Overflows here
        myTree.insertElement(myInt8, myInt8);

        Integer myInt9 = new Integer(53);
        myTree.insertElement(myInt9, myInt9);

        Integer myInt10 = new Integer(66);
        myTree.insertElement(myInt10, myInt10);

        Integer myInt11 = new Integer(19);
        myTree.insertElement(myInt11, myInt11);

        Integer myInt12 = new Integer(23);
        myTree.insertElement(myInt12, myInt12);

        Integer myInt13 = new Integer(24);
        myTree.insertElement(myInt13, myInt13);

        Integer myInt14 = new Integer(88);
        myTree.insertElement(myInt14, myInt14);

        Integer myInt15 = new Integer(1);
        myTree.insertElement(myInt15, myInt15);

        Integer myInt16 = new Integer(97);
        myTree.insertElement(myInt16, myInt16);

        Integer myInt17 = new Integer(94);
        myTree.insertElement(myInt17, myInt17);

        Integer myInt18 = new Integer(35);
        myTree.insertElement(myInt18, myInt18);

        Integer myInt19 = new Integer(51);
        myTree.insertElement(myInt19, myInt19);

        myTree.printAllElements();
        System.out.println("done");

        myTree.removeElement(24);
        myTree.removeElement(23);
        System.out.println("Done with removal.");


        myTree = new TwoFourTree(myComp);
        final int TEST_SIZE = 10000;


        for (int i = 0; i < TEST_SIZE; i++) {
            myTree.insertElement(new Integer(i), new Integer(i));
            //          myTree.printAllElements();
            //         myTree.checkTree();
        }
        System.out.println("removing");
        for (int i = 0; i < TEST_SIZE; i++) {
            int out = (Integer) myTree.removeElement(new Integer(i));
            if (out != i) {
                throw new TwoFourTreeException("main: wrong element removed");
            }
            if (i > TEST_SIZE - 15) {
                myTree.printAllElements();
            }
        }
        System.out.println("done");
    }

    public void printAllElements() {
        int indent = 0;
        if (root() == null) {
            System.out.println("The tree is empty");
        }
        else {
            printTree(root(), indent);
        }
    }

    public void printTree(TFNode start, int indent) {
        if (start == null) {
            return;
        }
        for (int i = 0; i < indent; i++) {
            System.out.print(" ");
        }
        printTFNode(start);
        indent += 4;
        int numChildren = start.getNumItems() + 1;
        for (int i = 0; i < numChildren; i++) {
            printTree(start.getChild(i), indent);
        }
    }

    public void printTFNode(TFNode node) {
        int numItems = node.getNumItems();
        for (int i = 0; i < numItems; i++) {
            System.out.print(((Item) node.getItem(i)).element() + " ");
        }
        System.out.println();
    }

    // checks if tree is properly hooked up, i.e., children point to parents
    public void checkTree() {
        checkTreeFromNode(treeRoot);
    }

    private void checkTreeFromNode(TFNode start) {
        if (start == null) {
            return;
        }

        if (start.getParent() != null) {
            TFNode parent = start.getParent();
            int childIndex = 0;
            for (childIndex = 0; childIndex <= parent.getNumItems(); childIndex++) {
                if (parent.getChild(childIndex) == start) {
                    break;
                }
            }
            // if child wasn't found, print problem
            if (childIndex > parent.getNumItems()) {
                System.out.println("Child to parent confusion");
                printTFNode(start);
            }
        }

        if (start.getChild(0) != null) {
            for (int childIndex = 0; childIndex <= start.getNumItems(); childIndex++) {
                if (start.getChild(childIndex) == null) {
                    System.out.println("Mixed null and non-null children");
                    printTFNode(start);
                }
                else {
                    if (start.getChild(childIndex).getParent() != start) {
                        System.out.println("Parent to child confusion");
                        printTFNode(start);
                    }
                    for (int i = childIndex - 1; i >= 0; i--) {
                        if (start.getChild(i) == start.getChild(childIndex)) {
                            System.out.println("Duplicate children of node");
                            printTFNode(start);
                        }
                    }
                }

            }
        }

        int numChildren = start.getNumItems() + 1;
        for (int childIndex = 0; childIndex < numChildren; childIndex++) {
            checkTreeFromNode(start.getChild(childIndex));
        }

    }
}
