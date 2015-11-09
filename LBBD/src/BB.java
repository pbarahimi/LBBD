import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

import gurobi.*;

public class BB {
	double obj;
	int status;
	double bestObj; 
	int cnt = 0;
	private GRBModel model;
	private PriorityQueue<BBNode> unexploredNodes = new PriorityQueue<BBNode>();
	private Set<BBNode> leafNodes = new HashSet<BBNode>();

	public BB(GRBModel model) throws GRBException {
		this.model = model;
	}
	
	private boolean isBetter(double x) throws GRBException{
		if (model.get(GRB.IntAttr.ModelSense) == 1){  // Minimization
			if (x < this.bestObj)return true;
			else return false;
		}else{
			if (x < this.bestObj)return false;
			else return true;
		}
	}

	public Set<BBNode> run() throws GRBException{
		BBNode root = new BBNode(0, model);
		int counter = 0;
		//initializing bestObj
		if (root.getModel().get(GRB.IntAttr.ModelSense) == 1)
			this.bestObj = 1 * GRB.INFINITY;
		else 
			this.bestObj = -1 * GRB.INFINITY;
				
		root.solve();
		counter++;
		unexploredNodes.add(root);
		
		//branch and bound
		while (!unexploredNodes.isEmpty()){
			BBNode node = unexploredNodes.peek();
//			System.out.println(unexploredNodes.size() + "-" + node.getIndex());			
			if (node.getStatus() == 2 && node.isIntegral()){  // feasible with integral solution
				leafNodes.add(node);
				if (isBetter(node.getObj())) bestObj = node.getObj();
				unexploredNodes.remove();
				System.out.println("# of leaves: " + leafNodes.size());
			}else if (node.getStatus() == 3){ // infeasible
				leafNodes.add(node);
				unexploredNodes.remove();
				System.out.println("# of leaves: " + leafNodes.size());
			}else if (node.getStatus() == 2 && !isBetter(node.getObj())){
				leafNodes.add(node);
				unexploredNodes.remove();
				System.out.println("# of leaves: " + leafNodes.size());
			}else if (node.getStatus() == 2 && !node.isIntegral()){ // nonintegral solution
				GRBVar var = node.getNonintVar();	// the variable to branch on.
				
				// creating left child node.
				BBNode childNodeLeft = new BBNode(2 * node.getIndex() + 1,  new GRBModel(node.getModel()));
				GRBLinExpr expr = new GRBLinExpr();
				expr.addTerm(1, var);
				childNodeLeft.getModel().addConstr(expr, GRB.GREATER_EQUAL, 1, "IntConst" + cnt++);
				childNodeLeft.solve(); counter++;
				childNodeLeft.getJ1().add(var);
				if (childNodeLeft.getStatus()==2) unexploredNodes.add(childNodeLeft);
				
				//creating right child node.
				BBNode childNodeRight = new BBNode(2 * node.getIndex() + 2, new GRBModel(node.getModel()));
				expr = new GRBLinExpr();
				expr.addTerm(1, var);
				childNodeRight.getModel().addConstr(expr, GRB.LESS_EQUAL, 0, "IntConst" + cnt++);
				childNodeRight.solve(); counter++;
				childNodeRight.getJ1().add(var);
				if (childNodeRight.getStatus()==2) unexploredNodes.add(childNodeRight);
				unexploredNodes.remove();
			}
		}
		System.out.println("The BB obj: " + bestObj);
		System.out.println("# of explored nodes: " + counter);
		System.out.println("# of leaves: " + leafNodes.size());
		
		return leafNodes;
	}
}