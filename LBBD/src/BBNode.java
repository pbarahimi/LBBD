

import java.util.List;

import gurobi.*;

public class BBNode implements Comparable<BBNode>{
	private int index;
	private double obj;
	private int status;
	private GRBModel model;
	private GRBVar[] vars;
	
	// comparator
	@Override
	public int compareTo(BBNode other) {
		return (int) (this.obj - other.obj);
	}
	
	// getters
	public GRBVar[] getVars(){return this.vars;}	
	public int getIndex(){return this.index;}	
	public double getObj(){return this.obj;}
	public GRBModel getModel(){return this.model;}
	public int getStatus(){return this.status;}
	
	// setter
	public void setIndex(int i){this.index=i;}
	public void setObj(double d){this.obj = d;}
	public void setStatus(int i){this.status = i;}
	public void setModel(GRBModel model){this.model = model;}
	
	// constructor
	public BBNode(int index, GRBModel model){
		this.index = index;
		this.model = model;
	}
	
	BBNode(BBNode other){
		this(other.index, other.model);
	}
	
	// solves the model and updates obj and status.
	public void solve() throws GRBException{
		this.model.optimize();	
		this.status = model.get(GRB.IntAttr.Status);
		if (this.status != 3){
			this.obj = model.get(GRB.DoubleAttr.ObjVal);
			this.vars = model.getVars();
		}
		/*
		if (this.getStatus()==2){
			for (GRBVar var : this.getVars()){
				System.out.println(var.get(GRB.StringAttr.VarName) + " " + var.get(GRB.DoubleAttr.X));
			}
		}*/
	}
	
	/**
	 * Checks if the solution is integral
	 * @return bool
	 * @throws GRBException
	 */
	public boolean isIntegral() throws GRBException{
		for (GRBVar var:model.getVars()){
			if (var.get(GRB.DoubleAttr.X)%1 != 0) return false;
		}
		return true;
	}

	/**
	 * Returns the first GRBVar with nonintegral value
	 * @return GRBVar
	 * @throws GRBException
	 */
	public GRBVar getNonintVar() throws GRBException{
		for (GRBVar var:model.getVars()){
			if (var.get(GRB.DoubleAttr.X)%1 != 0){
				System.out.println("Nonintegral variable is: " + var.get(GRB.StringAttr.VarName));
				return var;
			}
		}
		return null;
	}
}
