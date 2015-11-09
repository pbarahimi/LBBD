
import java.util.HashSet;
import java.util.Set;

import gurobi.*;

public class BBNode implements Comparable<BBNode>{
	private int index;
	private double obj;
	private boolean isIntegral = true;
	private int status;
	private GRBModel model;
	private Set<GRBVar> J0 = new HashSet<GRBVar>();
	private Set<GRBVar> J1 = new HashSet<GRBVar>();
//	private HashMap<String,Double> vars = new HashMap<String,Double>();
	
	// comparator
	@Override
	public int compareTo(BBNode other) {
		return (int) (this.obj - other.obj);
	}
	
	// getters
//	public HashMap<String,Double> getVars(){return this.vars;}	
	public int getIndex(){return this.index;}	
	public double getObj(){return this.obj;}
	public GRBModel getModel(){return this.model;}
	public int getStatus(){return this.status;}
	public boolean isIntegral(){return this.isIntegral;}
	public Set<GRBVar> getJ0(){return this.J0;}
	public Set<GRBVar> getJ1(){return this.J1;}
	
	// setter
	public void setIndex(int i){this.index=i;}
	public void setObj(double d){this.obj = d;}
	public void setStatus(int i){this.status = i;}
	public void setModel(GRBModel model){this.model = model;}
	
	// constructors
	public BBNode(int index, GRBModel model){
		this.index = index;
		this.model = model;
	}
	
	public BBNode(int index, GRBModel model, Set<GRBVar> J0, Set<GRBVar> J1){
		this.index = index;
		this.model = model;
		this.J0 = J0;
		this.J1 = J1;
	}
	
	BBNode(BBNode other){
		this(other.index, other.model, other.J0, other.J1);
	}
	
	// solves the model and updates obj and status.
	public void solve() throws GRBException{
		this.model.optimize();	
		this.status = model.get(GRB.IntAttr.Status);
		if (this.status != 3){
			this.obj = model.get(GRB.DoubleAttr.ObjVal);
			for (GRBVar var:model.getVars()){
				if (var.get(GRB.DoubleAttr.X)%1!=0) {this.isIntegral=false;continue;}
				//this.vars.put(var.get(GRB.StringAttr.VarName), var.get(GRB.DoubleAttr.X));
			}
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
/*	public boolean isIntegral() throws GRBException{
		for (double var:this.vars.values()){
			if (var%1 != 0) return false;
		}
		return true;
	}*/

	/**
	 * Returns the first GRBVar with nonintegral value
	 * @return GRBVar
	 * @throws GRBException
	 */
	public GRBVar getNonintVar() throws GRBException{
		for (GRBVar var:this.model.getVars()){
			if (var.get(GRB.DoubleAttr.X)%1 != 0){
//				System.out.println("Nonintegral: " + var.get(GRB.StringAttr.VarName));
				return var;
			}
		}model.remove(model.getConstrByName("y1"));
		return null;
	}
	
	/**
	 * Disposes the model to free memory
	 */
	public void disposeModel(){
		this.model = null;
	}
}