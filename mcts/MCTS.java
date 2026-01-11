/*
 * Copyright (c) 2021 by Damien Pellier <Damien.Pellier@imag.fr>.
 *
 * This file is part of PDDL4J library.
 *
 * PDDL4J is free software: you can redistribute it and/or modify * it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * PDDL4J is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License * along with PDDL4J.  If not,
 * see <http://www.gnu.org/licenses/>
 */

package fr.uga.pddl4j.examples.mcts;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import picocli.CommandLine;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import fr.uga.pddl4j.examples.mcts.Node;

/**
 * The class is an example. It shows how to create a simple MCTS search planner able to
 * solve an ADL problem by choosing the heuristic to used and its weight.
 *
 * @author D. Pellier
 * @version 4.0 - 30.11.2021
 */
@CommandLine.Command(name = "MCTS",
    version = "MCTS 1.?",
    description = "Solves a specified planning problem using MCTS search strategy.",
    sortOptions = false,
    mixinStandardHelpOptions = true,
    headerHeading = "Usage:%n",
    synopsisHeading = "%n",
    descriptionHeading = "%nDescription:%n%n",
    parameterListHeading = "%nParameters:%n",
    optionListHeading = "%nOptions:%n")
public class MCTS extends AbstractPlanner {

    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(MCTS.class.getName());

    /**
     * The HEURISTIC property used for planner configuration.
     */
    public static final String HEURISTIC_SETTING = "HEURISTIC";

    /**
     * The default value of the HEURISTIC property used for planner configuration.
     */
    public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.FAST_FORWARD;

    /**
     * The WEIGHT_HEURISTIC property used for planner configuration.
     */
    public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";

    /**
     * The default value of the WEIGHT_HEURISTIC property used for planner configuration.
     */
    public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;

    /**
     * The weight of the heuristic.
     */
    private double heuristicWeight;

    /**
     * The name of the heuristic used by the planner.
     */
    private StateHeuristic.Name heuristic;

    /**
     * Creates a new MCTS search planner with the default configuration.
     */
    public MCTS() {
        this(MCTS.getDefaultConfiguration());
    }

    /**
     * Creates a new MCTS search planner with a specified configuration.
     *
     * @param configuration the configuration of the planner.
     */
    public MCTS(final PlannerConfiguration configuration) {
        super();
        this.setConfiguration(configuration);
    }

    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
        paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    /**
     * Set the name of heuristic used by the planner to the solve a planning problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
        description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
            + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    /**
     * Returns the name of the heuristic used by the planner to solve a planning problem.
     *
     * @return the name of the heuristic used by the planner to solve a planning problem.
     */
    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    /**
     * Returns the weight of the heuristic.
     *
     * @return the weight of the heuristic.
     */
    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

    /**
     * Instantiates the planning problem from a parsed problem.
     *
     * @param problem the problem to instantiate.
     * @return the instantiated planning problem or null if the problem cannot be instantiated.
     */
    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    /**
     * Search a solution plan to a specified domain and problem using MCTS.
     *
     * @param problem the problem to solve.
     * @return the plan found or null if no plan was found.
     */
    @Override
    public Plan solve(final Problem problem) {
        LOGGER.info("* Starting MCTS search \n");
        final long begin = System.currentTimeMillis();
        try {
            Plan plan = this.mcts(problem);
            final long end = System.currentTimeMillis();
            
            if (plan != null) {
                LOGGER.info("* MCTS search succeeded\n");
                this.getStatistics().setTimeToSearch(end - begin);
            } else {
                LOGGER.info("* MCTS search failed\n");
            }
            return plan;
        } catch (ProblemNotSupportedException e) {
            LOGGER.fatal(e.getMessage());
            return null;
        }
    }

    /**
     * Checks the planner configuration and returns if the configuration is valid.
     * A configuration is valid if (1) the domain and the problem files exist and
     * can be read, (2) the timeout is greater than 0, (3) the weight of the
     * heuristic is greater than 0 and (4) the heuristic is a not null.
     *
     * @return <code>true</code> if the configuration is valid <code>false</code> otherwise.
     */
    public boolean hasValidConfiguration() {
        return super.hasValidConfiguration()
            && this.getHeuristicWeight() > 0.0
            && this.getHeuristic() != null;
    }

    /**
     * This method return the default arguments of the planner.
     *
     * @return the default arguments of the planner.
     * @see PlannerConfiguration
     */
    public static PlannerConfiguration getDefaultConfiguration() {
        PlannerConfiguration config = Planner.getDefaultConfiguration();
        config.setProperty(MCTS.HEURISTIC_SETTING, MCTS.DEFAULT_HEURISTIC.toString());
        config.setProperty(MCTS.WEIGHT_HEURISTIC_SETTING,
            Double.toString(MCTS.DEFAULT_WEIGHT_HEURISTIC));
        return config;
    }

    /**
     * Returns the configuration of the planner.
     *
     * @return the configuration of the planner.
     */
    @Override
    public PlannerConfiguration getConfiguration() {
        final PlannerConfiguration config = super.getConfiguration();
        config.setProperty(MCTS.HEURISTIC_SETTING, this.getHeuristic().toString());
        config.setProperty(MCTS.WEIGHT_HEURISTIC_SETTING, Double.toString(this.getHeuristicWeight()));
        return config;
    }

    /**
     * Sets the configuration of the planner. If a planner setting is not defined in
     * the specified configuration, the setting is initialized with its default value.
     *
     * @param configuration the configuration to set.
     */
    @Override
    public void setConfiguration(final PlannerConfiguration configuration) {
        super.setConfiguration(configuration);
        if (configuration.getProperty(MCTS.WEIGHT_HEURISTIC_SETTING) == null) {
            this.setHeuristicWeight(MCTS.DEFAULT_WEIGHT_HEURISTIC);
        } else {
            this.setHeuristicWeight(Double.parseDouble(configuration.getProperty(
                MCTS.WEIGHT_HEURISTIC_SETTING)));
        }
        if (configuration.getProperty(MCTS.HEURISTIC_SETTING) == null) {
            this.setHeuristic(MCTS.DEFAULT_HEURISTIC);
        } else {
            this.setHeuristic(StateHeuristic.Name.valueOf(configuration.getProperty(
                MCTS.HEURISTIC_SETTING)));
        }
    }

    /**
     * The main method of the <code>MCTS</code> planner.
     *
     * @param args the arguments of the command line.
     */
    public static void main(String[] args) {
        try {
            final MCTS planner = new MCTS();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }
    }

    /***************************************************************************************************************************************************/
    /***************************************************************************************************************************************************/
    /***************************************************************************************************************************************************/

public State mctsPureRandom(final State s, final Problem problem, StateHeuristic heuristic) {
        double hmin = Double.POSITIVE_INFINITY; // hmin <- INF
        State smin = null; // smin <- NULL

        final int NUM_WALK = 50;
        final int LENGTH_WALK = 50;

        Random rand = new Random();

        for(int i = 0; i < NUM_WALK; i++){
            State sP = new State(s); // s′ <- s

            for(int j = 0; j < LENGTH_WALK; j++){
                List<Action> ActApplicable = new ArrayList<>(); // A

                for(Action a : problem.getActions()){ // A <- ApplicableActions(s′)
                    if(a.isApplicable(sP)) ActApplicable.add(a);
                }

                if(ActApplicable.isEmpty()) break; // if A = φ

                Action a = ActApplicable.get(rand.nextInt(ActApplicable.size()));

                State next = new State(sP);
                for (ConditionalEffect ce : a.getConditionalEffects()) {
                    if (sP.satisfy(ce.getCondition())) {
                        next.apply(ce.getEffect());
                    }
                }
                sP = next;

                if (sP.satisfy(problem.getGoal())) {
                    return sP;
                }
            }

            double hval = heuristic.estimate(sP, problem.getGoal());
            if (hval < hmin) {
                hmin = hval;
                smin = sP;
            }
        }

        if(smin == null) return s;
        else return smin;
    }

    private boolean isDeadEnd(State s, Problem problem) {
        for (Action a : problem.getActions()) {
            if (a.isApplicable(s)) return false;
        }
        return true;
    }

    public Object[] mctsPureRandomWithPath(final State s, final Problem problem, StateHeuristic heuristic) {
        double hmin = Double.POSITIVE_INFINITY; // hmin <- INF
        State smin = null; // smin <- NULL
        List<Integer> bestPath = null;

        final int NUM_WALK = 1000;
        final int LENGTH_WALK = 20;
        Random rand = new Random();

        for(int i = 0; i < NUM_WALK; i++){
            State sP = new State(s); // s′ <- s
            List<Integer> currentPath = new ArrayList<>();

            for(int j = 0; j < LENGTH_WALK; j++){
                List<Action> ActApplicable = new ArrayList<>(); // A
                List<Integer> applicableIndices = new ArrayList<>();

                for(int idx = 0; idx < problem.getActions().size(); idx++){
                    Action a = problem.getActions().get(idx);
                    if(a.isApplicable(sP)) {
                        ActApplicable.add(a);
                        applicableIndices.add(idx);
                    }
                }

                if(ActApplicable.isEmpty()) break; // if A = φ

                int randIdx = rand.nextInt(applicableIndices.size());
                Action a = ActApplicable.get(randIdx);
                int actionIndex = applicableIndices.get(randIdx);
                currentPath.add(actionIndex);

                State next = new State(sP);
                for (ConditionalEffect ce : a.getConditionalEffects()) {
                    if (sP.satisfy(ce.getCondition())) {
                        next.apply(ce.getEffect());
                    }
                }
                sP = next;

                if (sP.satisfy(problem.getGoal())) {
                    return new Object[]{sP, currentPath};
                }
            }

            double hval = heuristic.estimate(sP, problem.getGoal());
            if (hval < hmin) {
                hmin = hval;
                smin = sP;
                bestPath = new ArrayList<>(currentPath);
            }
        }

        if(smin == null) return new Object[]{s, new ArrayList<>()};
        else return new Object[]{smin, bestPath};
    }

    /***************************************************************************************************************************************************/
    /***************************************************************************************************************************************************/
    /***************************************************************************************************************************************************/

    /**
     * Search a solution plan for a planning problem using an MCTS search strategy.
     *
     * @param problem the problem to solve.
     * @return a plan solution for the problem or null if there is no solution
     * @throws ProblemNotSupportedException if the problem to solve is not supported by the planner.
     */
    public Plan mcts(Problem problem) throws ProblemNotSupportedException {
        // Check if the problem is supported by the planner
        if (!this.isSupported(problem)) {
            throw new ProblemNotSupportedException("Problem not supported");
        }

        // First we create an instance of the heuristic to use to guide the search
        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);

        // We get the initial state from the planning problem
        final State init = new State(problem.getInitialState());

        // We initialize the closed list of nodes (store the nodes explored)
        final Set<Node> close = new HashSet<>();

        // We initialize the opened list to store the pending node according to function f
        final double weight = this.getHeuristicWeight();
        final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                double f1 = weight * n1.getHeuristic() + n1.getCost();
                double f2 = weight * n2.getHeuristic() + n2.getCost();
                return Double.compare(f1, f2);
            }
        });

        // We create the root node of the tree search
        final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));

        // We add the root to the list of pending nodes
        open.add(root);
        Plan plan = null;

        // We set the timeout in ms allocated to the search
        final int timeout = this.getTimeout() * 1000;
        long time = 0;

        // We start the search
        /*******************************************************************/
        /************************* zone à modifier *************************/
        /*******************************************************************/
        State s = init;
        Node bestNode = new Node(s);
        double hmin = heuristic.estimate(s, problem.getGoal());
        int counter = 0;
        final int MAX_STEPS = 100;
        long startTime = System.currentTimeMillis();

        while(!s.satisfy(problem.getGoal()) && (System.currentTimeMillis() - startTime) < timeout) {

            if (counter > MAX_STEPS || isDeadEnd(s, problem)) {
                s = init;
                counter = 0;
                continue;
            }

            Object[] result = this.mctsPureRandomWithPath(s, problem, heuristic);
            State sNext = (State) result[0];
            @SuppressWarnings("unchecked")
            List<Integer> actionPath = (List<Integer>) result[1];
            
            double hNext = heuristic.estimate(sNext, problem.getGoal());

            if (hNext < hmin) {
                bestNode = reconstructPath(init, actionPath, problem, heuristic);
                hmin = hNext;
                s = sNext;
                counter = 0;
            } else {
                counter++;
            }
        }

        if (bestNode != null) {
            return extractPlan(bestNode, problem);
        }
        return null;
    }

    /**
     * Reconstruit le Node COMPLET avec la séquence d'actions exacte.
     */
    private Node reconstructPath(State initState, List<Integer> actionPath, Problem problem, StateHeuristic heuristic) {
        State currentState = new State(initState);
        Node current = new Node(currentState, null, -1, 0.0, heuristic.estimate(initState, problem.getGoal()));
        
        for (int actionIdx : actionPath) {
            Action a = problem.getActions().get(actionIdx);
            
            State nextState = new State(currentState);
            for (ConditionalEffect ce : a.getConditionalEffects()) {
                if (currentState.satisfy(ce.getCondition())) {
                    nextState.apply(ce.getEffect());
                }
            }

            double newCost = current.getCost() + 1.0;
            int newDepth = current.getDepth() + 1;
            Node next = new Node(nextState, current, actionIdx, newCost, newDepth, 
                            heuristic.estimate(nextState, problem.getGoal()));
            currentState = nextState;
            current = next;
        }
        return current;
    }


    /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */
    private Plan extractPlan(final Node node, final Problem problem) {
        Node n = node;
        final Plan plan = new SequentialPlan();
        while (n.getAction() != -1) {
            final Action a = problem.getActions().get(n.getAction());
            plan.add(0, a);
            n = n.getParent();
        }
        return plan;
    }

    /**
     * Returns if a specified problem is supported by the planner. Just ADL problem can be solved by this planner.
     *
     * @param problem the problem to test.
     * @return <code>true</code> if the problem is supported <code>false</code> otherwise.
     */
    @Override
    public boolean isSupported(Problem problem) {
        return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
            || problem.getRequirements().contains(RequireKey.CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
            || problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
            || problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
            || problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
            || problem.getRequirements().contains(RequireKey.FLUENTS)
            || problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
            || problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
            || problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
            || problem.getRequirements().contains(RequireKey.PREFERENCES)
            || problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
            || problem.getRequirements().contains(RequireKey.HIERARCHY))
            ? false : true;
    }
}
