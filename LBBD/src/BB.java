import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

import gurobi.*;

public class BB {
	double obj;
//	int nodeCnt;
	int status;
	double bestObj; 
	int cnt = 0;
	private GRBModel model;
	private List<BBNode> exploredNodes = new ArrayList<BBNode>();
	private PriorityQueue<BBNode> unexploredNodes = new PriorityQueue<BBNode>();
	private List<BBNode> leafNodes = new ArrayList<BBNode>();

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

	public List<BBNode> run() throws GRBException{
		BBNode root = new BBNode(0, model);
		
		//initializing bestObj
		if (root.getModel().get(GRB.IntAttr.ModelSense) == 1)
			this.bestObj = 1 * GRB.INFINITY;
		else 
			this.bestObj = -1 * GRB.INFINITY;
				
		root.solve();
		unexploredNodes.add(root);
		
		//branch and bound
		while (!unexploredNodes.isEmpty()){
			BBNode node = unexploredNodes.poll();
			exploredNodes.add(node);
			
			if (node.getStatus() == 2 && isBetter(node.getObj()) && node.isIntegral()){  // feasible with integral solution
				leafNodes.add(node);
				bestObj = node.getObj();
			}else if (node.getStatus() == 3){ // infeasible
				leafNodes.add(node);
			}else if (node.getStatus() == 2 && !isBetter(node.getObj())){
				leafNodes.add(node);
			}else if (node.getStatus() == 2 && !node.isIntegral()){ // nonintegral solution
				GRBVar var = node.getNonintVar();	// the variable to branch on.
				
				// creating left child node.
				BBNode childNodeLeft = new BBNode(2 * node.getIndex() + 1,  new GRBModel(node.getModel()));
				GRBLinExpr expr = new GRBLinExpr();
				expr.addTerm(1, var);
				childNodeLeft.getModel().addConstr(expr, GRB.GREATER_EQUAL, 1, "IntConst" + cnt++);
				childNodeLeft.solve();
				unexploredNodes.add(childNodeLeft);
				
				//creating right child node.
				BBNode childNodeRight = new BBNode(2 * node.getIndex() + 2,  new GRBModel(node.getModel()));
				expr = new GRBLinExpr();
				expr.addTerm(-1, var);
				childNodeRight.getModel().addConstr(expr, GRB.GREATER_EQUAL, 0, "IntConst" + cnt++);
				childNodeRight.solve();
				unexploredNodes.add(childNodeRight);
				node.disposeModel();
			}
		}
		System.out.println("The BB obj: " + bestObj);
		System.out.println("# of leaves: " + leafNodes.size());
		return leafNodes;
	}
}